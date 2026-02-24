// C entry point - only calls into C++ via extern "C"
extern void app_init(void);

void app_main() {
    app_init();
}
