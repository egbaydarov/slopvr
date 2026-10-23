/* The slice of the Vulkan and Vulkan loader/layer ABI that vklayer.c uses, declared by hand (x86_64 Linux) so the layer
 * builds without the Vulkan headers.  Values and layouts follow vulkan_core.h / vk_layer.h and are pinned below. */
#ifndef SLOP_VK_H
#define SLOP_VK_H
#include <stdint.h>
#include <stddef.h>

#define VKAPI_CALL
typedef uint32_t VkFlags;
typedef struct VkInstance_T *VkInstance;
typedef struct VkPhysicalDevice_T *VkPhysicalDevice;
typedef struct VkDevice_T *VkDevice;
typedef struct VkDisplayKHR_T *VkDisplayKHR;         /* non-dispatchable handles are pointers on 64-bit */
typedef struct VkDisplayModeKHR_T *VkDisplayModeKHR;
typedef struct VkAllocationCallbacks VkAllocationCallbacks; /* only passed through */

typedef enum { VK_SUCCESS = 0, VK_INCOMPLETE = 5, VK_ERROR_INITIALIZATION_FAILED = -3, VK_RESULT_MAX_ENUM = 0x7FFFFFFF } VkResult;
typedef enum {
	VK_STRUCTURE_TYPE_LOADER_INSTANCE_CREATE_INFO = 47,
	VK_STRUCTURE_TYPE_LOADER_DEVICE_CREATE_INFO = 48,
	VK_STRUCTURE_TYPE_MAX_ENUM = 0x7FFFFFFF
} VkStructureType;
typedef enum { VK_LAYER_LINK_INFO = 0, VK_LOADER_DATA_CALLBACK = 1, VK_LOADER_LAYER_CREATE_DEVICE_CALLBACK = 2, VK_LOADER_FEATURES = 3 } VkLayerFunction;
typedef enum { LAYER_NEGOTIATE_UNINTIALIZED = 0, LAYER_NEGOTIATE_INTERFACE_STRUCT = 1 } VkNegotiateLayerStructType;

typedef void (VKAPI_CALL *PFN_vkVoidFunction)(void);
typedef PFN_vkVoidFunction (VKAPI_CALL *PFN_vkGetInstanceProcAddr)(VkInstance, const char *);
typedef PFN_vkVoidFunction (VKAPI_CALL *PFN_vkGetDeviceProcAddr)(VkDevice, const char *);
typedef PFN_vkVoidFunction (VKAPI_CALL *PFN_GetPhysicalDeviceProcAddr)(VkInstance, const char *);

typedef struct { uint32_t width, height; } VkExtent2D;
typedef struct { VkExtent2D visibleRegion; uint32_t refreshRate; } VkDisplayModeParametersKHR;
typedef struct { VkDisplayModeKHR displayMode; VkDisplayModeParametersKHR parameters; } VkDisplayModePropertiesKHR;
typedef struct { VkStructureType sType; void *pNext; VkDisplayModePropertiesKHR displayModeProperties; } VkDisplayModeProperties2KHR;

typedef struct {
	VkStructureType sType; const void *pNext; VkFlags flags; const void *pApplicationInfo;
	uint32_t enabledLayerCount; const char *const *ppEnabledLayerNames;
	uint32_t enabledExtensionCount; const char *const *ppEnabledExtensionNames;
} VkInstanceCreateInfo;
typedef struct {
	VkStructureType sType; const void *pNext; VkFlags flags;
	uint32_t queueCreateInfoCount; const void *pQueueCreateInfos;
	uint32_t enabledLayerCount; const char *const *ppEnabledLayerNames;
	uint32_t enabledExtensionCount; const char *const *ppEnabledExtensionNames;
	const void *pEnabledFeatures;
} VkDeviceCreateInfo;

typedef VkResult (VKAPI_CALL *PFN_vkCreateInstance)(const VkInstanceCreateInfo *, const VkAllocationCallbacks *, VkInstance *);
typedef void (VKAPI_CALL *PFN_vkDestroyInstance)(VkInstance, const VkAllocationCallbacks *);
typedef VkResult (VKAPI_CALL *PFN_vkCreateDevice)(VkPhysicalDevice, const VkDeviceCreateInfo *, const VkAllocationCallbacks *, VkDevice *);
typedef void (VKAPI_CALL *PFN_vkDestroyDevice)(VkDevice, const VkAllocationCallbacks *);
typedef VkResult (VKAPI_CALL *PFN_vkGetDisplayModePropertiesKHR)(VkPhysicalDevice, VkDisplayKHR, uint32_t *, VkDisplayModePropertiesKHR *);
typedef VkResult (VKAPI_CALL *PFN_vkGetDisplayModeProperties2KHR)(VkPhysicalDevice, VkDisplayKHR, uint32_t *, VkDisplayModeProperties2KHR *);

/* loader <-> layer (vk_layer.h) */
typedef struct VkLayerInstanceLink_ {
	struct VkLayerInstanceLink_ *pNext;
	PFN_vkGetInstanceProcAddr pfnNextGetInstanceProcAddr;
	PFN_GetPhysicalDeviceProcAddr pfnNextGetPhysicalDeviceProcAddr;
} VkLayerInstanceLink;
typedef struct VkLayerDeviceLink_ {
	struct VkLayerDeviceLink_ *pNext;
	PFN_vkGetInstanceProcAddr pfnNextGetInstanceProcAddr;
	PFN_vkGetDeviceProcAddr pfnNextGetDeviceProcAddr;
} VkLayerDeviceLink;
typedef struct {
	VkStructureType sType; const void *pNext; VkLayerFunction function;
	union { VkLayerInstanceLink *pLayerInfo; void *pfnSetInstanceLoaderData; struct { void *pfnLayerCreateDevice, *pfnLayerDestroyDevice; } layerDevice; VkFlags loaderFeatures; } u;
} VkLayerInstanceCreateInfo;
typedef struct {
	VkStructureType sType; const void *pNext; VkLayerFunction function;
	union { VkLayerDeviceLink *pLayerInfo; void *pfnSetDeviceLoaderData; } u;
} VkLayerDeviceCreateInfo;
typedef struct {
	VkNegotiateLayerStructType sType; void *pNext; uint32_t loaderLayerInterfaceVersion;
	PFN_vkGetInstanceProcAddr pfnGetInstanceProcAddr;
	PFN_vkGetDeviceProcAddr pfnGetDeviceProcAddr;
	PFN_GetPhysicalDeviceProcAddr pfnGetPhysicalDeviceProcAddr;
} VkNegotiateLayerInterface;

_Static_assert(sizeof(VkDisplayModePropertiesKHR) == 24 && sizeof(VkDisplayModeProperties2KHR) == 40, "vk layout");
_Static_assert(sizeof(VkInstanceCreateInfo) == 64 && sizeof(VkDeviceCreateInfo) == 72, "vk layout");
_Static_assert(sizeof(VkLayerInstanceCreateInfo) == 40 && offsetof(VkLayerInstanceCreateInfo, u) == 24, "vk layer layout");
_Static_assert(sizeof(VkLayerDeviceCreateInfo) == 32 && offsetof(VkLayerDeviceCreateInfo, u) == 24, "vk layer layout");
_Static_assert(sizeof(VkNegotiateLayerInterface) == 48 && offsetof(VkNegotiateLayerInterface, pfnGetInstanceProcAddr) == 24, "vk layer layout");
_Static_assert(sizeof(VkLayerInstanceLink) == 24 && sizeof(VkLayerDeviceLink) == 24, "vk layer layout");
#endif
