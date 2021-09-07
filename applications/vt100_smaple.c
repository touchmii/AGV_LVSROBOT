#include <rtthread.h>
#include "vt100.h"
#include <stdio.h>

static void show_ps2() {
    vt_hide_cursor;
    vt_clear();
    // vt_store_screen;
    vt_set_terminal_size(120, 54);
    vt_set_bg_color(VT_B_BLACK);
    vt_draw_box(0, 0, 120, 64, '-', '|', '+');
    vt_draw_str_at(30,3, " PS2 GamePad test!");

    vt_set_terminal_default_size();
    vt_move_to(0, 23);
    rt_kprintf("\n");
    uint32_t strr = "→";
    rt_kprintf(strr);
    rt_kprintf("→\r\n");

    vt_clear_attr();
    vt_show_cursor();

}
MSH_CMD_EXPORT(show_ps2, PS2 Demo)