//
// Created by madhuparna on 10/19/22.
//

#include "illixr/common_lock.hpp"
#include "illixr/plugin.hpp"
#include <android/log.h>

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "common-lock", __VA_ARGS__))

class common_lock_impl : public common_lock {
public:
    common_lock_impl(const phonebook* const pb)
            : sb{pb->lookup_impl<switchboard>()}{
        int ret1 = sem_init(&sem_monado, pshared, value);
        int ret2 = sem_init(&sem_illixr, pshared, value);
        LOGI("Semaphore initialized ret1 = %d ret2 = %d", ret1, ret2);
    }

    void get_lock() {
        lock.lock();
    }

    void release_lock() {
        lock.unlock();
    }

    void wait_illixr() {

        illixr_monado = 1;
        while(illixr_monado)
            ;
        return;
    }

    void wait_monado() {
        illixr_monado = 0;
        while(!illixr_monado)
            ;
        return;
    }

private:
    mutable std::atomic<bool>                                        first_time{true};
    const std::shared_ptr<switchboard>                               sb;
    bool illixr_monado = 0;
};

class common_lock_plugin : public plugin {
public:
    common_lock_plugin(const std::string& name, phonebook* pb)
            : plugin{name, pb} {
        pb->register_impl<common_lock>(
                std::static_pointer_cast<common_lock>(std::make_shared<common_lock_impl>(pb)));
    }
};

PLUGIN_MAIN(common_lock_plugin);