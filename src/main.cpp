#include "core/system_init.h"
#include "app/app_controller.h"

void setup() {
    system_init();
    app_init();

}

void loop() {
    app_loop();
}
