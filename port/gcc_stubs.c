/** @file gcc_stubs.c
*   @brief Minimal newlib glue for the arm-none-eabi-gcc build
*
*   sys_startup.c calls __libc_init_array() so that C++ static constructors
*   (for example the global Bridgetek_EVE2 display object) run before main().
*   newlib's __libc_init_array() ends by calling _init(), which normally comes
*   from crti.o/crtn.o - and those are excluded by -nostartfiles. The linker
*   script emits no .init/.fini content either, so an empty definition is both
*   sufficient and correct here.
*/

void _init(void)
{
}

void _fini(void)
{
}
