/* slopvr — OpenVR driver ABI in pure C.
 *
 * vrserver talks to drivers through C++ vtables (Itanium ABI). An object is
 * `{ const void *const *vtbl; ... }`; slot i of vtbl is virtual method i in
 * declaration order; `this` is the first argument. Methods returning structs
 * larger than 16 bytes receive a hidden result pointer *before* `this`.
 * Constants/offsets below were extracted from openvr_driver.h (SDK 2.x) and
 * are pinned with static asserts. */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#define OVR_STR_IServerTrackedDeviceProvider "IServerTrackedDeviceProvider_004"
#define OVR_STR_ITrackedDeviceServerDriver "ITrackedDeviceServerDriver_005"
#define OVR_STR_IVRDisplayComponent "IVRDisplayComponent_003"
#define OVR_STR_IVRServerDriverHost "IVRServerDriverHost_006"
#define OVR_STR_IVRProperties "IVRProperties_001"
#define OVR_STR_IVRSettings "IVRSettings_003"
#define OVR_STR_IVRDriverInput "IVRDriverInput_004"
#define OVR_STR_IVRDriverLog "IVRDriverLog_001"

typedef struct { double w, x, y, z; } HmdQuaternion_t;
typedef struct { float v[2]; } HmdVector2_t;
typedef struct { float v[3]; } HmdVector3_t;
typedef struct { float m[3][4]; } HmdMatrix34_t;
typedef struct { HmdVector2_t vTopLeft, vBottomRight; } HmdRect2_t;
typedef struct { float rfRed[2], rfGreen[2], rfBlue[2]; } DistortionCoordinates_t;

typedef struct {
	double poseTimeOffset;
	HmdQuaternion_t qWorldFromDriverRotation;
	double vecWorldFromDriverTranslation[3];
	HmdQuaternion_t qDriverFromHeadRotation;
	double vecDriverFromHeadTranslation[3];
	double vecPosition[3], vecVelocity[3], vecAcceleration[3];
	HmdQuaternion_t qRotation;
	double vecAngularVelocity[3], vecAngularAcceleration[3];
	int32_t result;
	bool poseIsValid, willDriftInYaw, shouldApplyHeadModel, deviceIsConnected;
} DriverPose_t;
static_assert(sizeof(DriverPose_t) == 280, "DriverPose_t");

typedef struct { bool bActive, bValid, bTracked; HmdVector3_t vGazeOrigin, vGazeTarget; } VREyeTrackingData_t;
static_assert(sizeof(VREyeTrackingData_t) == 28, "VREyeTrackingData_t");

typedef struct __attribute__((packed, aligned(4))) {
	uint64_t containerHandle, componentHandle;
	float fDurationSeconds, fFrequency, fAmplitude;
} VREvent_HapticVibration_t;

typedef struct __attribute__((packed, aligned(4))) {
	uint32_t eventType;
	uint32_t trackedDeviceIndex;
	float eventAgeSeconds;
	uint8_t data[48];
} VREvent_t;
static_assert(sizeof(VREvent_t) == 60, "VREvent_t");

typedef struct { int32_t prop, writeType, eSetError; void *pvBuffer; uint32_t unBufferSize, unTag; int32_t eError; } PropertyWrite_t;
static_assert(sizeof(PropertyWrite_t) == 40, "PropertyWrite_t");

enum {
	VRInitError_None = 0, VRInitError_Init_InterfaceNotFound = 105, VRInitError_Driver_Failed = 200,
	TrackedDeviceClass_HMD = 1, TrackedDeviceClass_Controller = 2,
	TrackingResult_Uninitialized = 1, TrackingResult_Calibrating_InProgress = 100,
	TrackingResult_Running_OK = 200, TrackingResult_Running_OutOfRange = 201,
	Eye_Left = 0, Eye_Right = 1,
	TrackedControllerRole_LeftHand = 1, TrackedControllerRole_RightHand = 2,
	k_unFloatPropertyTag = 1, k_unInt32PropertyTag = 2, k_unUint64PropertyTag = 3, k_unBoolPropertyTag = 4,
	k_unStringPropertyTag = 5, k_unDoublePropertyTag = 7, k_unHmdMatrix34PropertyTag = 20,
	PropertyWrite_Set = 0,
	VRScalarType_Absolute = 0, VRScalarUnits_NormalizedOneSided = 0, VRScalarUnits_NormalizedTwoSided = 1,
	VREvent_TrackedDeviceUserInteractionStarted = 103, VREvent_TrackedDeviceUserInteractionEnded = 104,
	VREvent_SteamVRSectionSettingChanged = 857, VREvent_Input_HapticVibration = 1700,
};

enum {
	Prop_TrackingSystemName_String = 1000, Prop_ModelNumber_String = 1001, Prop_SerialNumber_String = 1002,
	Prop_RenderModelName_String = 1003, Prop_WillDriftInYaw_Bool = 1004, Prop_ManufacturerName_String = 1005,
	Prop_TrackingFirmwareVersion_String = 1006, Prop_HardwareRevision_String = 1007,
	Prop_DeviceIsWireless_Bool = 1010, Prop_DeviceIsCharging_Bool = 1011, Prop_DeviceBatteryPercentage_Float = 1012,
	Prop_FirmwareVersion_Uint64 = 1018, Prop_ContainsProximitySensor_Bool = 1025,
	Prop_DeviceProvidesBatteryStatus_Bool = 1026, Prop_DeviceCanPowerOff_Bool = 1027,
	Prop_DeviceClass_Int32 = 1029, Prop_HasCamera_Bool = 1030, Prop_DriverVersion_String = 1031,
	Prop_RegisteredDeviceType_String = 1036, Prop_InputProfilePath_String = 1037, Prop_NeverTracked_Bool = 1038,
	Prop_Identifiable_Bool = 1043,
	Prop_ReportsTimeSinceVSync_Bool = 2000, Prop_SecondsFromVsyncToPhotons_Float = 2001,
	Prop_DisplayFrequency_Float = 2002, Prop_UserIpdMeters_Float = 2003, Prop_CurrentUniverseId_Uint64 = 2004,
	Prop_IsOnDesktop_Bool = 2007, Prop_EdidVendorID_Int32 = 2011, Prop_EdidProductID_Int32 = 2015,
	Prop_UserHeadToEyeDepthMeters_Float = 2026, Prop_DisplayAllowNightMode_Bool = 2037,
	Prop_DisplayDebugMode_Bool = 2044, Prop_ExpectedTrackingReferenceCount_Int32 = 2049,
	Prop_ExpectedControllerCount_Int32 = 2050, Prop_DistortionMeshResolution_Int32 = 2056,
	Prop_DriverIsDrawingControllers_Bool = 2057, Prop_MinimumIpdStepMeters_Float = 2060,
	Prop_DriverProvidedChaperoneVisibility_Bool = 2076,
	Prop_DisplayAvailableFrameRates_Float_Array = 2080, Prop_DisplaySupportsMultipleFramerates_Bool = 2081, Prop_DisplaySupportsRuntimeFramerateChange_Bool = 2084,
	Prop_DisplaySupportsAnalogGain_Bool = 2085, Prop_DisplayMinAnalogGain_Float = 2086, Prop_DisplayMaxAnalogGain_Float = 2087,
	Prop_Hmd_SupportsHDCP14LegacyCompat_Bool = 2102, Prop_Hmd_SupportsDisplayPortTrainingMode_Bool = 2104,
	Prop_Hmd_SupportsAppThrottling_Bool = 2106, Prop_Hmd_AllowSupersampleFiltering_Bool = 2115,
	Prop_Audio_DefaultPlaybackDeviceId_String = 2300, Prop_Audio_DefaultRecordingDeviceId_String = 2301,
	Prop_ControllerRoleHint_Int32 = 3007,
	Prop_NamedIconPathDeviceOff_String = 5001, Prop_NamedIconPathDeviceSearching_String = 5002,
	Prop_NamedIconPathDeviceSearchingAlert_String = 5003, Prop_NamedIconPathDeviceReady_String = 5004,
	Prop_NamedIconPathDeviceReadyAlert_String = 5005, Prop_NamedIconPathDeviceNotReady_String = 5006,
	Prop_NamedIconPathDeviceStandby_String = 5007, Prop_NamedIconPathDeviceAlertLow_String = 5008,
	Prop_HasDisplayComponent_Bool = 6002, Prop_HasControllerComponent_Bool = 6003,
	Prop_HasCameraComponent_Bool = 6004, Prop_HasDriverDirectModeComponent_Bool = 6005,
	Prop_HasVirtualDisplayComponent_Bool = 6006,
	Prop_ControllerType_String = 7000, Prop_ControllerHandSelectionPriority_Int32 = 7002,
};

/* ---- host interfaces (vrserver implements, we call) ---- */
typedef void (*ovr_fn)(void);
typedef struct { const ovr_fn *vt; } ovr_obj;

#define OVR_CALL(obj, idx, ret, ...) ((ret (*)(void *, ##__VA_ARGS__))((obj)->vt[idx]))

/* IVRDriverContext */
static inline void *ovr_ctx_GetGenericInterface(ovr_obj *o, const char *v, int32_t *err)
{ return OVR_CALL(o, 0, void *, const char *, int32_t *)(o, v, err); }
static inline uint64_t ovr_ctx_GetDriverHandle(ovr_obj *o) { return OVR_CALL(o, 1, uint64_t)(o); }

/* IVRServerDriverHost_006 */
static inline bool ovr_host_TrackedDeviceAdded(ovr_obj *o, const char *serial, int32_t cls, void *drv)
{ return OVR_CALL(o, 0, bool, const char *, int32_t, void *)(o, serial, cls, drv); }
static inline void ovr_host_TrackedDevicePoseUpdated(ovr_obj *o, uint32_t id, const DriverPose_t *p)
{ OVR_CALL(o, 1, void, uint32_t, const DriverPose_t *, uint32_t)(o, id, p, sizeof *p); }
static inline void ovr_host_SetDisplayEyeToHead(ovr_obj *o, uint32_t id, const HmdMatrix34_t *l, const HmdMatrix34_t *r)
{ OVR_CALL(o, 9, void, uint32_t, const HmdMatrix34_t *, const HmdMatrix34_t *)(o, id, l, r); }
static inline bool ovr_host_IsExiting(ovr_obj *o) { return OVR_CALL(o, 4, bool)(o); }
static inline bool ovr_host_PollNextEvent(ovr_obj *o, VREvent_t *e)
{ return OVR_CALL(o, 5, bool, VREvent_t *, uint32_t)(o, e, sizeof *e); }
static inline void ovr_host_SetDisplayProjectionRaw(ovr_obj *o, uint32_t id, const HmdRect2_t *l, const HmdRect2_t *r)
{ OVR_CALL(o, 10, void, uint32_t, const HmdRect2_t *, const HmdRect2_t *)(o, id, l, r); }

/* IVRProperties_001 */
static inline int32_t ovr_props_WritePropertyBatch(ovr_obj *o, uint64_t c, PropertyWrite_t *b, uint32_t n)
{ return OVR_CALL(o, 1, int32_t, uint64_t, PropertyWrite_t *, uint32_t)(o, c, b, n); }
static inline uint64_t ovr_props_TrackedDeviceToPropertyContainer(ovr_obj *o, uint32_t id)
{ return OVR_CALL(o, 3, uint64_t, uint32_t)(o, id); }

/* IVRSettings_003 */
static inline void ovr_settings_SetInt32(ovr_obj *o, const char *s, const char *k, int32_t v, int32_t *err)
{ OVR_CALL(o, 2, void, const char *, const char *, int32_t, int32_t *)(o, s, k, v, err); }
static inline void ovr_settings_SetFloat(ovr_obj *o, const char *s, const char *k, float v, int32_t *err)
{ OVR_CALL(o, 3, void, const char *, const char *, float, int32_t *)(o, s, k, v, err); }
static inline bool ovr_settings_GetBool(ovr_obj *o, const char *s, const char *k, int32_t *err)
{ return OVR_CALL(o, 5, bool, const char *, const char *, int32_t *)(o, s, k, err); }
static inline int32_t ovr_settings_GetInt32(ovr_obj *o, const char *s, const char *k, int32_t *err)
{ return OVR_CALL(o, 6, int32_t, const char *, const char *, int32_t *)(o, s, k, err); }
static inline float ovr_settings_GetFloat(ovr_obj *o, const char *s, const char *k, int32_t *err)
{ return OVR_CALL(o, 7, float, const char *, const char *, int32_t *)(o, s, k, err); }

/* IVRDriverInput_004 */
static inline int32_t ovr_input_CreateBooleanComponent(ovr_obj *o, uint64_t c, const char *n, uint64_t *h)
{ return OVR_CALL(o, 0, int32_t, uint64_t, const char *, uint64_t *)(o, c, n, h); }
static inline int32_t ovr_input_UpdateBooleanComponent(ovr_obj *o, uint64_t h, bool v, double t)
{ return OVR_CALL(o, 1, int32_t, uint64_t, bool, double)(o, h, v, t); }
static inline int32_t ovr_input_CreateScalarComponent(ovr_obj *o, uint64_t c, const char *n, uint64_t *h, int32_t type, int32_t units)
{ return OVR_CALL(o, 2, int32_t, uint64_t, const char *, uint64_t *, int32_t, int32_t)(o, c, n, h, type, units); }
static inline int32_t ovr_input_UpdateScalarComponent(ovr_obj *o, uint64_t h, float v, double t)
{ return OVR_CALL(o, 3, int32_t, uint64_t, float, double)(o, h, v, t); }
static inline int32_t ovr_input_CreateHapticComponent(ovr_obj *o, uint64_t c, const char *n, uint64_t *h)
{ return OVR_CALL(o, 4, int32_t, uint64_t, const char *, uint64_t *)(o, c, n, h); }
static inline int32_t ovr_input_CreatePoseComponent(ovr_obj *o, uint64_t c, const char *n, uint64_t *h)
{ return OVR_CALL(o, 7, int32_t, uint64_t, const char *, uint64_t *)(o, c, n, h); }
static inline int32_t ovr_input_UpdatePoseComponent(ovr_obj *o, uint64_t h, const HmdMatrix34_t *m, double t)
{ return OVR_CALL(o, 8, int32_t, uint64_t, const HmdMatrix34_t *, double)(o, h, m, t); }
static inline int32_t ovr_input_CreateEyeTrackingComponent(ovr_obj *o, uint64_t c, const char *n, uint64_t *h)
{ return OVR_CALL(o, 9, int32_t, uint64_t, const char *, uint64_t *)(o, c, n, h); }
static inline int32_t ovr_input_UpdateEyeTrackingComponent(ovr_obj *o, uint64_t h, const VREyeTrackingData_t *d, double t)
{ return OVR_CALL(o, 10, int32_t, uint64_t, const VREyeTrackingData_t *, double)(o, h, d, t); }

/* IVRDriverLog_001 */
static inline void ovr_log_Log(ovr_obj *o, const char *msg) { OVR_CALL(o, 0, void, const char *)(o, msg); }

/* ---- interfaces we implement (vrserver calls) ----
 * A vtable is an array of function pointers; slots [0],[1] are offset-to-top
 * and RTTI (unused by vrserver, kept for ABI hygiene); the object's vptr
 * points at slot [2]. */
#define OVR_VTABLE(name, ...) static const ovr_fn name##_raw[] = { 0, 0, __VA_ARGS__ }; \
	static const ovr_fn *const name = name##_raw + 2
/* OVR_FN(T, f): f must have exactly the signature T (checked by the ?: operand rule). */
#define OVR_FN(T, f) ((ovr_fn)(1 ? (f) : (T)0))

/* IServerTrackedDeviceProvider_004 */
typedef int32_t (*ovr_provider_Init)(void *self, ovr_obj *ctx);
typedef void (*ovr_provider_Cleanup)(void *self);
typedef const char *const *(*ovr_provider_GetInterfaceVersions)(void *self);
typedef void (*ovr_provider_RunFrame)(void *self);
typedef bool (*ovr_provider_ShouldBlockStandbyMode)(void *self);
typedef void (*ovr_provider_EnterStandby)(void *self);
typedef void (*ovr_provider_LeaveStandby)(void *self);

/* ITrackedDeviceServerDriver_005 */
typedef int32_t (*ovr_dev_Activate)(void *self, uint32_t id);
typedef void (*ovr_dev_Deactivate)(void *self);
typedef void (*ovr_dev_EnterStandby)(void *self);
typedef void *(*ovr_dev_GetComponent)(void *self, const char *name);
typedef void (*ovr_dev_DebugRequest)(void *self, const char *req, char *resp, uint32_t len);
typedef void (*ovr_dev_GetPose)(DriverPose_t *sret, void *self);

/* IVRDisplayComponent_003 */
typedef void (*ovr_disp_GetWindowBounds)(void *self, int32_t *x, int32_t *y, uint32_t *w, uint32_t *h);
typedef bool (*ovr_disp_IsDisplayOnDesktop)(void *self);
typedef bool (*ovr_disp_IsDisplayRealDisplay)(void *self);
typedef void (*ovr_disp_GetRecommendedRenderTargetSize)(void *self, uint32_t *w, uint32_t *h);
typedef void (*ovr_disp_GetEyeOutputViewport)(void *self, int32_t eye, uint32_t *x, uint32_t *y, uint32_t *w, uint32_t *h);
typedef void (*ovr_disp_GetProjectionRaw)(void *self, int32_t eye, float *l, float *r, float *t, float *b);
typedef void (*ovr_disp_ComputeDistortion)(DistortionCoordinates_t *sret, void *self, int32_t eye, float u, float v);
typedef bool (*ovr_disp_ComputeInverseDistortion)(void *self, HmdVector2_t *out, int32_t eye, uint32_t ch, float u, float v);
