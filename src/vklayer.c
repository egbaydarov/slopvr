/* slopvr Vulkan implicit layer: works around a vrcompositor bug.
 *
 * vrcompositor picks the DRM display mode closest to steamvr.preferredRefreshRate but only
 * records the mode's refresh rate when a *later* mode beats an earlier one; if the best mode is
 * the first one with the right resolution, it keeps its 90 Hz default and tells apps (and its own
 * frame timing) the panel runs at 90 Hz. The PSVR2 reports 4000x2040@120 before 4000x2040@90, so
 * 120 Hz always hit that path. Sorting each resolution's modes by ascending refresh rate makes the
 * higher rate a "later, better" match. Does nothing outside vrcompositor.
 */
#include "vk.h"
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define EXPORT __attribute__((visibility("default")))

typedef struct {
	void *key; /* loader dispatch table pointer, shared by VkInstance and its VkPhysicalDevices */
	VkInstance instance;
	PFN_vkGetInstanceProcAddr gipa;
} Inst;
typedef struct { void *key; PFN_vkGetDeviceProcAddr gdpa; } Dev;

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static Inst g_inst[16];
static Dev g_dev[16];
static int g_ninst, g_ndev;
static bool g_active; /* only rewrite modes inside vrcompositor */

__attribute__((constructor)) static void init(void)
{
	char comm[32] = { 0 };
	FILE *f = fopen("/proc/self/comm", "r");
	if (f) { if (!fgets(comm, sizeof comm, f)) comm[0] = 0; fclose(f); }
	g_active = !strncmp(comm, "vrcompositor", 12);
}

static Inst *inst_of(void *handle)
{
	void *key = *(void **)handle;
	for (int i = 0; i < g_ninst; i++) if (g_inst[i].key == key) return &g_inst[i];
	return NULL;
}
static Dev *dev_of(void *handle)
{
	void *key = *(void **)handle;
	for (int i = 0; i < g_ndev; i++) if (g_dev[i].key == key) return &g_dev[i];
	return NULL;
}

static int cmp_mode(const void *a, const void *b)
{
	const VkDisplayModeParametersKHR *x = &((const VkDisplayModePropertiesKHR *)a)->parameters;
	const VkDisplayModeParametersKHR *y = &((const VkDisplayModePropertiesKHR *)b)->parameters;
	if (x->visibleRegion.width != y->visibleRegion.width) return x->visibleRegion.width < y->visibleRegion.width ? -1 : 1;
	if (x->visibleRegion.height != y->visibleRegion.height) return x->visibleRegion.height < y->visibleRegion.height ? -1 : 1;
	return x->refreshRate < y->refreshRate ? -1 : x->refreshRate > y->refreshRate;
}
static int cmp_mode2(const void *a, const void *b)
{
	return cmp_mode(&((const VkDisplayModeProperties2KHR *)a)->displayModeProperties,
	                &((const VkDisplayModeProperties2KHR *)b)->displayModeProperties);
}

static VkResult VKAPI_CALL l_GetDisplayModePropertiesKHR(VkPhysicalDevice pd, VkDisplayKHR disp, uint32_t *n, VkDisplayModePropertiesKHR *p)
{
	Inst *in = inst_of(pd);
	if (!in) return VK_ERROR_INITIALIZATION_FAILED;
	PFN_vkGetDisplayModePropertiesKHR next = (PFN_vkGetDisplayModePropertiesKHR)in->gipa(in->instance, "vkGetDisplayModePropertiesKHR");
	VkResult r = next(pd, disp, n, p);
	if (p && *n > 1 && (r == VK_SUCCESS || r == VK_INCOMPLETE)) qsort(p, *n, sizeof *p, cmp_mode);
	return r;
}
static VkResult VKAPI_CALL l_GetDisplayModeProperties2KHR(VkPhysicalDevice pd, VkDisplayKHR disp, uint32_t *n, VkDisplayModeProperties2KHR *p)
{
	Inst *in = inst_of(pd);
	if (!in) return VK_ERROR_INITIALIZATION_FAILED;
	PFN_vkGetDisplayModeProperties2KHR next = (PFN_vkGetDisplayModeProperties2KHR)in->gipa(in->instance, "vkGetDisplayModeProperties2KHR");
	VkResult r = next(pd, disp, n, p);
	if (p && *n > 1 && (r == VK_SUCCESS || r == VK_INCOMPLETE)) qsort(p, *n, sizeof *p, cmp_mode2);
	return r;
}

static VkResult VKAPI_CALL l_CreateInstance(const VkInstanceCreateInfo *ci, const VkAllocationCallbacks *ac, VkInstance *out)
{
	VkLayerInstanceCreateInfo *li = (VkLayerInstanceCreateInfo *)ci->pNext;
	while (li && !(li->sType == VK_STRUCTURE_TYPE_LOADER_INSTANCE_CREATE_INFO && li->function == VK_LAYER_LINK_INFO))
		li = (VkLayerInstanceCreateInfo *)li->pNext;
	if (!li || !li->u.pLayerInfo) return VK_ERROR_INITIALIZATION_FAILED;
	PFN_vkGetInstanceProcAddr gipa = li->u.pLayerInfo->pfnNextGetInstanceProcAddr;
	li->u.pLayerInfo = li->u.pLayerInfo->pNext; /* advance the chain for the next layer */
	PFN_vkCreateInstance create = (PFN_vkCreateInstance)gipa(NULL, "vkCreateInstance");
	VkResult r = create(ci, ac, out);
	if (r != VK_SUCCESS) return r;
	pthread_mutex_lock(&g_lock);
	if (g_ninst < (int)(sizeof g_inst / sizeof *g_inst))
		g_inst[g_ninst++] = (Inst){ *(void **)*out, *out, gipa };
	pthread_mutex_unlock(&g_lock);
	return r;
}
static void VKAPI_CALL l_DestroyInstance(VkInstance inst, const VkAllocationCallbacks *ac)
{
	Inst *in = inst_of(inst);
	if (!in) return;
	PFN_vkDestroyInstance next = (PFN_vkDestroyInstance)in->gipa(inst, "vkDestroyInstance");
	pthread_mutex_lock(&g_lock);
	*in = g_inst[--g_ninst];
	pthread_mutex_unlock(&g_lock);
	next(inst, ac);
}

static VkResult VKAPI_CALL l_CreateDevice(VkPhysicalDevice pd, const VkDeviceCreateInfo *ci, const VkAllocationCallbacks *ac, VkDevice *out)
{
	Inst *in = inst_of(pd);
	VkLayerDeviceCreateInfo *li = (VkLayerDeviceCreateInfo *)ci->pNext;
	while (li && !(li->sType == VK_STRUCTURE_TYPE_LOADER_DEVICE_CREATE_INFO && li->function == VK_LAYER_LINK_INFO))
		li = (VkLayerDeviceCreateInfo *)li->pNext;
	if (!in || !li || !li->u.pLayerInfo) return VK_ERROR_INITIALIZATION_FAILED;
	PFN_vkGetDeviceProcAddr gdpa = li->u.pLayerInfo->pfnNextGetDeviceProcAddr;
	li->u.pLayerInfo = li->u.pLayerInfo->pNext;
	PFN_vkCreateDevice create = (PFN_vkCreateDevice)in->gipa(in->instance, "vkCreateDevice");
	VkResult r = create(pd, ci, ac, out);
	if (r != VK_SUCCESS) return r;
	pthread_mutex_lock(&g_lock);
	if (g_ndev < (int)(sizeof g_dev / sizeof *g_dev)) g_dev[g_ndev++] = (Dev){ *(void **)*out, gdpa };
	pthread_mutex_unlock(&g_lock);
	return r;
}
static void VKAPI_CALL l_DestroyDevice(VkDevice dev, const VkAllocationCallbacks *ac)
{
	Dev *d = dev_of(dev);
	if (!d) return;
	PFN_vkDestroyDevice next = (PFN_vkDestroyDevice)d->gdpa(dev, "vkDestroyDevice");
	pthread_mutex_lock(&g_lock);
	*d = g_dev[--g_ndev];
	pthread_mutex_unlock(&g_lock);
	next(dev, ac);
}

static PFN_vkVoidFunction VKAPI_CALL l_GetDeviceProcAddr(VkDevice dev, const char *name)
{
	if (!strcmp(name, "vkGetDeviceProcAddr")) return (PFN_vkVoidFunction)l_GetDeviceProcAddr;
	if (!strcmp(name, "vkDestroyDevice")) return (PFN_vkVoidFunction)l_DestroyDevice;
	Dev *d = dev_of(dev);
	return d ? d->gdpa(dev, name) : NULL;
}
static PFN_vkVoidFunction VKAPI_CALL l_GetInstanceProcAddr(VkInstance inst, const char *name)
{
	if (!strcmp(name, "vkGetInstanceProcAddr")) return (PFN_vkVoidFunction)l_GetInstanceProcAddr;
	if (!strcmp(name, "vkCreateInstance")) return (PFN_vkVoidFunction)l_CreateInstance;
	if (!strcmp(name, "vkDestroyInstance")) return (PFN_vkVoidFunction)l_DestroyInstance;
	if (!strcmp(name, "vkCreateDevice")) return (PFN_vkVoidFunction)l_CreateDevice;
	if (!strcmp(name, "vkGetDeviceProcAddr")) return (PFN_vkVoidFunction)l_GetDeviceProcAddr;
	if (g_active) {
		if (!strcmp(name, "vkGetDisplayModePropertiesKHR")) return (PFN_vkVoidFunction)l_GetDisplayModePropertiesKHR;
		if (!strcmp(name, "vkGetDisplayModeProperties2KHR")) return (PFN_vkVoidFunction)l_GetDisplayModeProperties2KHR;
	}
	if (!inst) return NULL;
	Inst *in = inst_of(inst);
	return in ? in->gipa(inst, name) : NULL;
}

EXPORT VkResult VKAPI_CALL vkNegotiateLoaderLayerInterfaceVersion(VkNegotiateLayerInterface *v)
{
	if (v->loaderLayerInterfaceVersion < 2) return VK_ERROR_INITIALIZATION_FAILED;
	v->loaderLayerInterfaceVersion = 2;
	v->pfnGetInstanceProcAddr = l_GetInstanceProcAddr;
	v->pfnGetDeviceProcAddr = l_GetDeviceProcAddr;
	v->pfnGetPhysicalDeviceProcAddr = NULL;
	return VK_SUCCESS;
}
