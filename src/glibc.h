/* The driver runs inside the Steam Linux Runtime (sniper, glibc 2.31) but is built on whatever glibc the host has.  A
 * modern glibc hands out newer symbol versions for a few functions: pthread_create and pthread_join moved into libc in
 * 2.34, strtol and the scanf family got C23 variants in 2.38, fmod and hypot got new entry points.  The old versions still
 * exist in every glibc as compatibility symbols, so bind to them explicitly.  stat() cannot be pinned (before 2.33 it was an
 * inline wrapper) and is not used.  build.sh checks the result: no symbol newer than GLIBC_2.31 may remain. */
#if defined(__x86_64__) && defined(__linux__)
__asm__(".symver fmod,fmod@GLIBC_2.2.5");
__asm__(".symver hypot,hypot@GLIBC_2.2.5");
__asm__(".symver pthread_create,pthread_create@GLIBC_2.2.5");
__asm__(".symver pthread_join,pthread_join@GLIBC_2.2.5");
__asm__(".symver __isoc23_strtol,strtol@GLIBC_2.2.5");
__asm__(".symver __isoc23_strtoul,strtoul@GLIBC_2.2.5");
__asm__(".symver __isoc23_fscanf,__isoc99_fscanf@GLIBC_2.7");
__asm__(".symver __isoc23_sscanf,__isoc99_sscanf@GLIBC_2.7");
#endif
