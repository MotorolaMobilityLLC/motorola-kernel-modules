#ifndef __HOOK_HELPER_H__
#define __HOOK_HELPER_H__

#include <trace/hooks/sched.h>

#define REGISTER_ANDROID_VH_HOOK(name) do {\
	rc = register_trace_android_vh_##name(name##_hook, NULL);\
	if (rc) {\
		pr_err("register android vh hook %s failed", #name);\
		goto err_out_##name;\
	}\
} while (0)

#define UNREGISTER_ANDROID_VH_HOOK(name) do {\
	unregister_trace_android_vh_##name(name##_hook, NULL);\
} while (0)

#define REGISTER_ANDROID_VH_ALIAS_HOOK(name, hook) do {\
	rc = register_trace_android_vh_##name(hook, NULL);\
	if (rc) {\
		pr_err("register android vh hook %s failed", #name);\
		goto err_out_##name;\
	}\
} while (0)

#define UNREGISTER_ANDROID_VH_ALIAS_HOOK(name, hook) do {\
	unregister_trace_android_vh_##name(hook, NULL);\
} while (0)

#define ERROR_OUT(name) err_out_##name

#endif
