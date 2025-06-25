/*
 * Copyright (c) 2025 Motorola Inc.
 */

#ifndef _MOTO_MDPF_SYSFS_H_
#define _MOTO_MDPF_SYSFS_H_

#define MAX_PROC_SIZE 128

#if IS_ENABLED(CONFIG_MOTO_ENABLE_MDPF)
int mdpf_proc_init(void);
void mdpf_proc_deinit(void);
#else
int mdpf_proc_init(void){
	return 0;
}
void mdpf_proc_deinit(void){
}
#endif

#endif /* _MOTO_MDPF_SYSFS_H_ */
