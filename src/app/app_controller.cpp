#include "app_controller.h"
#include "../drivers/led_rgb.h"
#include "tasks.h"

extern LedRGB led;  // instance globale si nécessaire

void app_init() {

    // Lancement de la task A2DP
    xTaskCreate(
        task_bl_a2dp_stream,   // fonction task
        "A2DP Stream",          // nom (debug)
        4096,                   // stack (ok pour BT)
        NULL,                   // paramètre
        5,                      // priorité (BT = moyen/haut)
        NULL                    // handle (inutile ici)
    );
}

void app_loop() {
    // logique applicative simple
    // ex : LED, état système, etc.
}
