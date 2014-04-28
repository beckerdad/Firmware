// James has hacked this to make a data logging application.

#ifndef SDLOG2_VERSION_H_
#define SDLOG2_VERSION_H_

/*
 GIT_VERSION is defined at build time via a Makefile call to the
 git command line.
 */
#define FREEZE_STR(s) #s
#define STRINGIFY(s) FREEZE_STR(s)
#define FW_GIT STRINGIFY(GIT_VERSION)

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
#define	HW_ARCH "PX4FMU_V1"
#endif

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V2
#define	HW_ARCH "PX4FMU_V2"
#endif

#endif /* SDLOG2_VERSION_H_ */
