#ifndef IPU4_COMPAT_H
#define IPU4_COMPAT_H

#include <linux/version.h>
#include <media/media-entity.h>
#include <media/v4l2-subdev.h>

#include <linux/slab.h>
#ifndef kzalloc_obj
// Typed allocation helpers of 7.0 (backported to 6.12.y and 6.18.y).
#define kzalloc_obj(P) ((typeof(P) *)kzalloc(sizeof(P), GFP_KERNEL))
#define kmalloc_objs(P, COUNT) \
	((typeof(P) *)kmalloc_array(COUNT, sizeof(P), GFP_KERNEL))
#endif

#if KERNEL_VERSION(6, 18, 0) > LINUX_VERSION_CODE
// https://github.com/torvalds/linux/commit/683342ce3c0dae068bf0ee157ee12c13088193f7
static __maybe_unused s64 ambu_v4l2_get_link_freq(struct media_pad *src_pad, unsigned int mul,
		       unsigned int div) {
    struct v4l2_subdev *ext_sd = media_entity_to_v4l2_subdev(src_pad->entity);
	
    if (WARN(!ext_sd, "Failed to get subdev"))
		return -ENODEV;

	return v4l2_get_link_freq(ext_sd->ctrl_handler, 0, 0);
}
#undef v4l2_get_link_freq
#define v4l2_get_link_freq(PAD, MUL, DIV) ambu_v4l2_get_link_freq(PAD, MUL, DIV)
#endif

#if KERNEL_VERSION(6, 13, 0) > LINUX_VERSION_CODE
// Symbol namespaces are string literals since 6.13; older kernels
// stringify a bare token.
#include <linux/export.h>
#include <linux/module.h>
#undef EXPORT_SYMBOL_NS_GPL
#define EXPORT_SYMBOL_NS_GPL(sym, ns)	__EXPORT_SYMBOL(sym, "GPL", ns)
#undef MODULE_IMPORT_NS
#define MODULE_IMPORT_NS(ns)	MODULE_INFO(import_ns, ns)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 14, 0)
// Before 6.14 this was X86_MATCH_VFM_STEPPINGS(), taking a stepping mask
#define X86_MATCH_VFM_STEPS(vfm, min_step, max_step, data) \
	X86_MATCH_VFM_STEPPINGS(vfm, GENMASK(max_step, min_step), data)
#endif // 6.14

#endif // IPU4_COMPAT_H
