// NetworkTables publisher / subscriber for the Limelight vision server.
//
// Recovered from Ghidra decompilation of FUN_00194440 (CommInterface_NT
// constructor) in the original visionserver binary. The 42 topic names
// below are the exact strings found in the .rodata section and
// correspond 1:1 to the topics registered by the original code.
//
// Table naming convention (from FUN_0020c320 / getGlobalSettings):
//     usb_id == 0    ->  /limelight/
//     usb_id != 0    ->  /limelight-<usb_id>/
//
// The original binary uses ntcore's C++ API (confirmed by the presence of
// _ZN2nt12NetworkTable* symbols in .dynsym). We mirror that here.

#include "visionserver/nt_publisher.h"

#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/StringTopic.h>
#include <networktables/StringArrayTopic.h>

#include <spdlog/spdlog.h>

#include <array>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace limelight {

namespace {

// Exact 42 topic name constants recovered from the binary. Keep as
// constexpr to catch typos at compile time and give link-time deduplication.
// Inputs (robot -> LL)
constexpr const char* kInPipeline             = "pipeline";
constexpr const char* kInLedMode              = "ledMode";
constexpr const char* kInCrosshairs           = "crosshairs";
constexpr const char* kInStream               = "stream";
constexpr const char* kInSnapshot             = "snapshot";
constexpr const char* kInPriorityId           = "priorityid";
constexpr const char* kInLlRobot              = "llrobot";
constexpr const char* kInLlPython             = "llpython";
constexpr const char* kInRobotOrientation     = "robot_orientation_set";
constexpr const char* kInCamPoseRobotSpaceSet = "camerapose_robotspace_set";
constexpr const char* kInFiducialIdFilters    = "fiducial_id_filters_set";
constexpr const char* kInFiducialDownscale    = "fiducial_downscale_set";
constexpr const char* kInFiducialOffset       = "fiducial_offset_set";
constexpr const char* kInImuMode              = "imumode_set";
constexpr const char* kInImuAssistAlpha       = "imuassistalpha_set";
constexpr const char* kInThrottle             = "throttle_set";
constexpr const char* kInKeystone             = "keystone_set";
constexpr const char* kInRewindEnable         = "rewind_enable_set";
constexpr const char* kInCaptureRewind        = "capture_rewind";

// Outputs (LL -> robot)
constexpr const char* kOutGetPipe          = "getpipe";
constexpr const char* kOutGetPipeType      = "getpipetype";
constexpr const char* kOutJson             = "json";
constexpr const char* kOutTcornxy          = "tcornxy";
constexpr const char* kOutRawTargets       = "rawtargets";
constexpr const char* kOutRawFiducials     = "rawfiducials";
constexpr const char* kOutRawDetections    = "rawdetections";
constexpr const char* kOutRawBarcodes      = "rawbarcodes";
constexpr const char* kOutTcClass          = "tcclass";
constexpr const char* kOutTdClass          = "tdclass";
constexpr const char* kOutStdDevs          = "stddevs";
constexpr const char* kOutCamPoseRS        = "camerapose_robotspace";
constexpr const char* kOutCamPoseTS        = "camerapose_targetspace";
constexpr const char* kOutTargetPoseCS     = "targetpose_cameraspace";
constexpr const char* kOutTargetPoseRS     = "targetpose_robotspace";
constexpr const char* kOutBotpose          = "botpose";
constexpr const char* kOutBotposeRed       = "botpose_wpired";
constexpr const char* kOutBotposeBlue      = "botpose_wpiblue";
constexpr const char* kOutBotposeOrb       = "botpose_orb";
constexpr const char* kOutBotposeOrbRed    = "botpose_orb_wpired";
constexpr const char* kOutBotposeOrbBlue   = "botpose_orb_wpiblue";
constexpr const char* kOutBotposeTargetSp  = "botpose_targetspace";

// Standard single-target fields. Not in FUN_00194440 — they're registered
// per-pipeline in the target output publisher, but we hold the handles here.
constexpr const char* kOutTx   = "tx";
constexpr const char* kOutTy   = "ty";
constexpr const char* kOutTa   = "ta";
constexpr const char* kOutTs   = "ts";
constexpr const char* kOutTid  = "tid";
constexpr const char* kOutTv   = "tv";
constexpr const char* kOutTl   = "tl";   // pipeline latency
constexpr const char* kOutCl   = "cl";   // capture latency
constexpr const char* kOutHb   = "hb";   // heartbeat

// Determine the NT table name the same way the original binary does.
std::string deriveTableName(int usb_id) {
    if (usb_id == 0) return "limelight";
    return "limelight-" + std::to_string(usb_id);
}

}  // namespace

// ---------------------------------------------------------------------------

struct NTPublisher::Impl {
    std::shared_ptr<nt::NetworkTable> table;
    nt::NetworkTableInstance inst;

    // Output publishers — held so we don't have to re-fetch handles every
    // frame. ntcore is thread-safe but GetDoubleTopic() isn't free.
    nt::DoublePublisher      pubGetPipe;
    nt::StringPublisher      pubGetPipeType;
    nt::StringPublisher      pubJson;
    nt::DoubleArrayPublisher pubTcornxy;
    nt::DoubleArrayPublisher pubRawTargets;
    nt::DoubleArrayPublisher pubRawFiducials;
    nt::DoubleArrayPublisher pubRawDetections;
    nt::StringArrayPublisher pubRawBarcodes;
    nt::StringPublisher      pubTcClass;
    nt::StringPublisher      pubTdClass;
    nt::DoubleArrayPublisher pubStdDevs;
    nt::DoubleArrayPublisher pubCamPoseRS;
    nt::DoubleArrayPublisher pubCamPoseTS;
    nt::DoubleArrayPublisher pubTargetPoseCS;
    nt::DoubleArrayPublisher pubTargetPoseRS;
    nt::DoubleArrayPublisher pubBotpose;
    nt::DoubleArrayPublisher pubBotposeRed;
    nt::DoubleArrayPublisher pubBotposeBlue;
    nt::DoubleArrayPublisher pubBotposeOrb;
    nt::DoubleArrayPublisher pubBotposeOrbRed;
    nt::DoubleArrayPublisher pubBotposeOrbBlue;
    nt::DoubleArrayPublisher pubBotposeTargetSp;

    // Single-target (per-frame) publishers.
    nt::DoublePublisher pubTx, pubTy, pubTa, pubTs, pubTv, pubTl, pubCl, pubHb;
    nt::DoublePublisher pubTid;

    // Input subscribers. For _set topics the robot writes and we listen via
    // NT listeners; ntcore calls our callback on its network thread.
    //
    // We MUST hold the subscriber handles for the lifetime of the publisher
    // or ntcore tears the underlying NT_Subscriber down and listeners stop
    // firing. Initial implementation had these as locals inside start(); the
    // listeners "worked" briefly then went silent.
    std::vector<nt::DoubleSubscriber>      doubleSubs;
    std::vector<nt::DoubleArraySubscriber> doubleArraySubs;
    std::vector<NT_Listener> listeners;

    Callbacks cb;
};

// ---------------------------------------------------------------------------

NTPublisher::NTPublisher(const std::string& nt_server, int team_number, int usb_id)
    : impl_(std::make_unique<Impl>()),
      table_name_(deriveTableName(usb_id)),
      team_number_(team_number),
      nt_server_(nt_server) {}

NTPublisher::~NTPublisher() { stop(); }

void NTPublisher::setCallbacks(Callbacks cb) {
    std::lock_guard<std::mutex> lock(mutex_);
    impl_->cb = std::move(cb);
}

bool NTPublisher::start() {
    impl_->inst = nt::NetworkTableInstance::GetDefault();

    // Original binary starts as a client4 — NT4 client mode, addressable by
    // either IP (nt_server) or by FRC team number. If nt_server is a valid
    // IP string, use it verbatim; otherwise fall back to team number.
    impl_->inst.SetServer(nt_server_.c_str(), NT_DEFAULT_PORT4);
    impl_->inst.StartClient4(table_name_.c_str());
    if (team_number_ > 0) {
        impl_->inst.SetServerTeam(static_cast<unsigned int>(team_number_));
    }

    impl_->table = impl_->inst.GetTable(table_name_);

    // ------- Output publishers --------------------------------------------
    auto dbl  = [&](const char* name) {
        return impl_->table->GetDoubleTopic(name).Publish();
    };
    auto darr = [&](const char* name) {
        return impl_->table->GetDoubleArrayTopic(name).Publish();
    };
    auto str  = [&](const char* name) {
        return impl_->table->GetStringTopic(name).Publish();
    };
    auto sarr = [&](const char* name) {
        return impl_->table->GetStringArrayTopic(name).Publish();
    };

    impl_->pubGetPipe          = dbl(kOutGetPipe);
    impl_->pubGetPipeType      = str(kOutGetPipeType);
    impl_->pubJson             = str(kOutJson);
    impl_->pubTcornxy          = darr(kOutTcornxy);
    impl_->pubRawTargets       = darr(kOutRawTargets);
    impl_->pubRawFiducials     = darr(kOutRawFiducials);
    impl_->pubRawDetections    = darr(kOutRawDetections);
    impl_->pubRawBarcodes      = sarr(kOutRawBarcodes);
    impl_->pubTcClass          = str(kOutTcClass);
    impl_->pubTdClass          = str(kOutTdClass);
    impl_->pubStdDevs          = darr(kOutStdDevs);
    impl_->pubCamPoseRS        = darr(kOutCamPoseRS);
    impl_->pubCamPoseTS        = darr(kOutCamPoseTS);
    impl_->pubTargetPoseCS     = darr(kOutTargetPoseCS);
    impl_->pubTargetPoseRS     = darr(kOutTargetPoseRS);
    impl_->pubBotpose          = darr(kOutBotpose);
    impl_->pubBotposeRed       = darr(kOutBotposeRed);
    impl_->pubBotposeBlue      = darr(kOutBotposeBlue);
    impl_->pubBotposeOrb       = darr(kOutBotposeOrb);
    impl_->pubBotposeOrbRed    = darr(kOutBotposeOrbRed);
    impl_->pubBotposeOrbBlue   = darr(kOutBotposeOrbBlue);
    impl_->pubBotposeTargetSp  = darr(kOutBotposeTargetSp);

    impl_->pubTx  = dbl(kOutTx);
    impl_->pubTy  = dbl(kOutTy);
    impl_->pubTa  = dbl(kOutTa);
    impl_->pubTs  = dbl(kOutTs);
    impl_->pubTid = dbl(kOutTid);
    impl_->pubTv  = dbl(kOutTv);
    impl_->pubTl  = dbl(kOutTl);
    impl_->pubCl  = dbl(kOutCl);
    impl_->pubHb  = dbl(kOutHb);

    // ------- Input listeners ----------------------------------------------
    // Helper: add a listener on a topic that fires cb(value) on update.
    auto subDbl = [&](const char* name, auto&& handler) {
        impl_->doubleSubs.push_back(
            impl_->table->GetDoubleTopic(name).Subscribe(0.0));
        auto l = impl_->inst.AddListener(
            impl_->doubleSubs.back(), nt::EventFlags::kValueAll,
            [h = std::forward<decltype(handler)>(handler)](const nt::Event& e) {
                if (e.GetValueEventData())
                    h(e.GetValueEventData()->value.GetDouble());
            });
        impl_->listeners.push_back(l);
    };
    auto subDArr = [&](const char* name, auto&& handler) {
        impl_->doubleArraySubs.push_back(
            impl_->table->GetDoubleArrayTopic(name).Subscribe({}));
        auto l = impl_->inst.AddListener(
            impl_->doubleArraySubs.back(), nt::EventFlags::kValueAll,
            [h = std::forward<decltype(handler)>(handler)](const nt::Event& e) {
                if (auto* v = e.GetValueEventData()) {
                    auto span = v->value.GetDoubleArray();
                    std::vector<double> vec(span.begin(), span.end());
                    h(std::move(vec));
                }
            });
        impl_->listeners.push_back(l);
    };

    // Re-fetching impl_->cb inside each listener protects against a caller
    // swapping the Callbacks struct via setCallbacks() after start().  The
    // original implementation copied the std::function by reference, so a
    // post-start reassignment left dangling slots that threw bad_function_call.
    Impl* ip = impl_.get();
    auto hasCb  = [ip](auto Callbacks::* m) { return static_cast<bool>(ip->cb.*m); };
    auto fireD  = [ip](auto Callbacks::* m, double v) {
        if (ip->cb.*m) (ip->cb.*m)(v);
    };
    auto fireI  = [ip](auto Callbacks::* m, int v) {
        if (ip->cb.*m) (ip->cb.*m)(v);
    };
    auto fireV  = [ip](auto Callbacks::* m, std::vector<double> v) {
        if (ip->cb.*m) (ip->cb.*m)(std::move(v));
    };
    auto fireA6 = [ip](auto Callbacks::* m, std::vector<double> v) {
        if (ip->cb.*m) {
            std::array<double, 6> arr{};
            for (size_t i = 0; i < std::min<size_t>(6, v.size()); ++i) arr[i] = v[i];
            (ip->cb.*m)(arr);
        }
    };

    if (hasCb(&Callbacks::on_pipeline))     subDbl(kInPipeline,   [fireI](double v) { fireI(&Callbacks::on_pipeline,    static_cast<int>(v)); });
    if (hasCb(&Callbacks::on_led_mode))     subDbl(kInLedMode,    [fireI](double v) { fireI(&Callbacks::on_led_mode,    static_cast<int>(v)); });
    if (hasCb(&Callbacks::on_crosshairs))   subDbl(kInCrosshairs, [fireI](double v) { fireI(&Callbacks::on_crosshairs,  static_cast<int>(v)); });
    if (hasCb(&Callbacks::on_stream))       subDbl(kInStream,     [fireI](double v) { fireI(&Callbacks::on_stream,      static_cast<int>(v)); });
    if (hasCb(&Callbacks::on_snapshot))     subDbl(kInSnapshot,   [fireI](double v) { fireI(&Callbacks::on_snapshot,    static_cast<int>(v)); });
    if (hasCb(&Callbacks::on_priority_id))  subDbl(kInPriorityId, [fireI](double v) { fireI(&Callbacks::on_priority_id, static_cast<int>(v)); });
    if (hasCb(&Callbacks::on_llrobot))      subDArr(kInLlRobot,   [fireV](std::vector<double> v) { fireV(&Callbacks::on_llrobot,  std::move(v)); });
    if (hasCb(&Callbacks::on_llpython))     subDArr(kInLlPython,  [fireV](std::vector<double> v) { fireV(&Callbacks::on_llpython, std::move(v)); });
    if (hasCb(&Callbacks::on_fiducial_id_filters))
        subDArr(kInFiducialIdFilters, [fireV](std::vector<double> v) { fireV(&Callbacks::on_fiducial_id_filters, std::move(v)); });
    if (hasCb(&Callbacks::on_fiducial_downscale))
        subDbl(kInFiducialDownscale, [fireD](double v) { fireD(&Callbacks::on_fiducial_downscale, v); });
    if (hasCb(&Callbacks::on_fiducial_offset))
        subDArr(kInFiducialOffset, [fireV](std::vector<double> v) { fireV(&Callbacks::on_fiducial_offset, std::move(v)); });
    if (hasCb(&Callbacks::on_imu_mode))         subDbl(kInImuMode,        [fireI](double v) { fireI(&Callbacks::on_imu_mode,      static_cast<int>(v)); });
    if (hasCb(&Callbacks::on_imu_assist_alpha)) subDbl(kInImuAssistAlpha, [fireD](double v) { fireD(&Callbacks::on_imu_assist_alpha, v); });
    if (hasCb(&Callbacks::on_throttle))         subDbl(kInThrottle,       [fireD](double v) { fireD(&Callbacks::on_throttle,      v); });
    if (hasCb(&Callbacks::on_keystone))         subDArr(kInKeystone,      [fireV](std::vector<double> v) { fireV(&Callbacks::on_keystone, std::move(v)); });
    if (hasCb(&Callbacks::on_rewind_enable))    subDbl(kInRewindEnable,   [fireI](double v) { fireI(&Callbacks::on_rewind_enable, static_cast<int>(v)); });
    if (hasCb(&Callbacks::on_capture_rewind))   subDbl(kInCaptureRewind,  [fireI](double v) { fireI(&Callbacks::on_capture_rewind, static_cast<int>(v)); });

    if (hasCb(&Callbacks::on_robot_orientation))
        subDArr(kInRobotOrientation, [fireA6](std::vector<double> v) { fireA6(&Callbacks::on_robot_orientation, std::move(v)); });
    if (hasCb(&Callbacks::on_camerapose_robotspace))
        subDArr(kInCamPoseRobotSpaceSet, [fireA6](std::vector<double> v) { fireA6(&Callbacks::on_camerapose_robotspace, std::move(v)); });

    spdlog::info("NT: client4 connected to {} (team={}), table=/{}",
                 nt_server_, team_number_, table_name_);
    return true;
}

void NTPublisher::stop() {
    if (!impl_) return;
    for (auto l : impl_->listeners) {
        impl_->inst.RemoveListener(l);
    }
    impl_->listeners.clear();
    impl_->doubleSubs.clear();
    impl_->doubleArraySubs.clear();
    if (impl_->inst) {
        impl_->inst.StopClient();
    }
}

void NTPublisher::publish(const PipelineResult& result,
                          int active_pipeline_index,
                          const std::string& active_pipeline_type,
                          double capture_latency_ms,
                          double pipeline_latency_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!impl_->table) return;

    // --- Metadata --------------------------------------------------------
    impl_->pubGetPipe.Set(static_cast<double>(active_pipeline_index));
    impl_->pubGetPipeType.Set(active_pipeline_type);

    // --- Single-target scalars ------------------------------------------
    impl_->pubTx.Set(result.tx);
    impl_->pubTy.Set(result.ty);
    impl_->pubTa.Set(result.ta);
    impl_->pubTs.Set(result.ts);
    impl_->pubTid.Set(static_cast<double>(result.tid));
    impl_->pubTv.Set(result.has_target ? 1.0 : 0.0);
    impl_->pubTl.Set(pipeline_latency_ms);
    impl_->pubCl.Set(capture_latency_ms);

    // --- Botpose variants ------------------------------------------------
    if (!result.botpose.empty())      impl_->pubBotpose.Set(result.botpose);
    if (!result.botpose_red.empty())  impl_->pubBotposeRed.Set(result.botpose_red);
    if (!result.botpose_blue.empty()) impl_->pubBotposeBlue.Set(result.botpose_blue);

    // MegaTag2 / IMU-fused botpose.  Only populated when a tag was matched
    // against the field map and an IMU yaw source was available for this
    // frame (see FiducialPipeline::process).
    if (!result.botpose_orb.empty())
        impl_->pubBotposeOrb.Set(result.botpose_orb);
    if (!result.botpose_orb_wpired.empty())
        impl_->pubBotposeOrbRed.Set(result.botpose_orb_wpired);
    if (!result.botpose_orb_wpiblue.empty())
        impl_->pubBotposeOrbBlue.Set(result.botpose_orb_wpiblue);

    // --- Target-space poses ----------------------------------------------
    if (!result.targetpose_cameraspace.empty())
        impl_->pubTargetPoseCS.Set(result.targetpose_cameraspace);
    if (!result.targetpose_robotspace.empty())
        impl_->pubTargetPoseRS.Set(result.targetpose_robotspace);

    // --- Corners (flattened) ---------------------------------------------
    if (!result.corners.empty()) {
        std::vector<double> flat;
        flat.reserve(result.corners.size() * 2);
        for (const auto& pt : result.corners) {
            for (double c : pt) flat.push_back(c);
        }
        impl_->pubTcornxy.Set(flat);
    }

    // --- Class labels (from classifier/detector) -------------------------
    if (!result.tclass.empty()) {
        impl_->pubTcClass.Set(result.tclass);
        impl_->pubTdClass.Set(result.tclass);
    }

    // --- Heartbeat -------------------------------------------------------
    publishHeartbeat();
}

void NTPublisher::publishHeartbeat() {
    uint64_t h = heartbeat_.fetch_add(1, std::memory_order_relaxed) + 1;
    impl_->pubHb.Set(static_cast<double>(h));
}

void NTPublisher::publishJson(const std::string& json_payload) {
    std::lock_guard<std::mutex> lock(mutex_);
    impl_->pubJson.Set(json_payload);
}

}  // namespace limelight
