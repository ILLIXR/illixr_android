#include "switchboard.hpp"

using namespace ILLIXR;

#ifdef ILLIXR_ANDROID_BUILD
AAssetManager* switchboard::asset_manager_ = nullptr;
#endif

switchboard::topic_subscription::topic_subscription(const std::string& topic_name, plugin_id_t plugin_id,
                                                    std::function<void(ptr<const event>&&, std::size_t)> callback,
                                                    const std::shared_ptr<record_logger>& record_logger_)
        : topic_name_{topic_name}
        , plugin_id_{plugin_id}
        , callback_{std::move(callback)}
        , record_logger_{record_logger_}
        , cb_log_{record_logger_}
        , thread_{[this] {
            this->thread_body();
        },
                  [] {
                      thread_on_start();
                  },
                  [this] {
                      this->thread_on_stop();
                  }} {
    thread_.start();
}

void switchboard::topic_subscription::thread_body() {
    // Try to pull event off of queue
    ptr<const event> this_event;
    std::int64_t     timeout_usecs = std::chrono::duration_cast<std::chrono::microseconds>(queue_timeout_).count();
    // Note the use of timed blocking wait
    if (queue_.wait_dequeue_timed(token_, this_event, timeout_usecs)) {
        // Process event
        // Also, record and log the time
        dequeued_++;
        auto cb_start_cpu_time  = thread_cpu_time();
        auto cb_start_wall_time = std::chrono::high_resolution_clock::now();
        // std::cerr << "deq " << ptr_to_str(reinterpret_cast<const void*>(this_event.get_ro())) << " " <<
        // this_event.use_count() << " v\n";
        callback_(std::move(this_event), dequeued_);
        if (cb_log_) {
            cb_log_.log(record{_switchboard_callback_header,
                               {
                                       {plugin_id_},
                                       {topic_name_},
                                       {dequeued_},
                                       {cb_start_cpu_time},
                                       {thread_cpu_time()},
                                       {cb_start_wall_time},
                                       {std::chrono::high_resolution_clock::now()},
                               }});
        }
    } else {
        // Nothing to do.
        idle_cycles_++;
    }
}

void switchboard::topic_subscription::thread_on_stop() {
    // Drain queue
    std::size_t unprocessed = enqueued_ - dequeued_;
    {
        ptr<const event> this_event;
        for (std::size_t i = 0; i < unprocessed; ++i) {
            [[maybe_unused]] bool ret = queue_.try_dequeue(token_, this_event);
            assert(ret);
            // std::cerr << "deq (stopping) " << ptr_to_str(reinterpret_cast<const void*>(this_event.get_ro())) << " "
            // << this_event.use_count() << " v\n";
            this_event.reset();
        }
    }

    // Log stats
    if (record_logger_) {
        record_logger_->log(record{_switchboard_topic_stop_header,
                                   {
                                           {plugin_id_},
                                           {topic_name_},
                                           {dequeued_},
                                           {unprocessed},
                                           {idle_cycles_},
                                   }});
    }
}

void switchboard::topic::put(ptr<const ILLIXR::switchboard::event> && this_event) {
    assert(this_event != nullptr);
    assert(this_event.unique() ||
           this_event.use_count() <= 2); /// <-- TODO: Revisit for solution that guarantees uniqueness

    /* The pointer that this gets exchanged with needs to get dropped. */
    size_t index          = (latest_index_.load() + 1) % latest_buffer_size_;
    latest_buffer_[index] = this_event;
    latest_index_++;

    // Read/write on subscriptions_.
    // Must acquire shared state on subscriptions_lock_
    std::unique_lock lock{subscriptions_lock_};
    for (topic_subscription& ts : subscriptions_) {
        // std::cerr << "enq " << ptr_to_str(reinterpret_cast<const void*>(this_event->get())) << " " <<
        // this_event->use_count() << " ^\n";
        ptr<const event> event_ptr_copy{this_event};
        ts.enqueue(std::move(event_ptr_copy));
    }

    for (topic_buffer& ts : buffers_) {
        // std::cerr << "enq " << ptr_to_str(reinterpret_cast<const void*>(this_event->get())) << " " <<
        // this_event->use_count() << " ^\n";
        ptr<const event> event_ptr_copy{this_event};
        ts.enqueue(std::move(event_ptr_copy));
    }
    // std::cerr << "put done " << ptr_to_str(reinterpret_cast<const void*>(this_event->get())) << " " <<
    // this_event->use_count() << " (= 1 + len(sub)) \n";
}

[[maybe_unused]] void switchboard::topic::deserialize_and_put(std::vector<char>& buffer,
                                                              network::topic_config& config) {
    if (config.serialization_method == network::topic_config::SerializationMethod::BOOST) {
        // TODO: Need to differentiate and support protobuf deserialization
        boost::iostreams::stream<boost::iostreams::array_source> stream{buffer.data(), buffer.size()};
        boost::archive::binary_iarchive                          ia{stream};
        ptr<event>                                               this_event;
        ia >> this_event;
        put(std::move(this_event));
    } else {
        ptr<event> message = std::make_shared<event_wrapper<std::string>>((std::string(buffer.begin(), buffer.end())));
        put(std::move(message));
    }
}

switchboard::switchboard(const ILLIXR::phonebook* pb)
        : phonebook_{pb}
        , record_logger_{pb ? pb->lookup_impl<record_logger>() : nullptr} {
    for (const auto& item : ENV_VARS) {
        char* value = getenv(item.c_str());
        if (value) {
            env_vars_[item] = value;
        } else {
            env_vars_[item] = "";
        }
    }
}

[[maybe_unused]] bool switchboard::topic_exists(const std::string& topic_name) {
    const std::shared_lock lock{registry_lock_};
    auto                   found = registry_.find(topic_name);
    return found != registry_.end();
}

[[maybe_unused]] switchboard::topic& switchboard::get_topic(const std::string& topic_name) {
    const std::shared_lock lock{registry_lock_};
    auto                   found = registry_.find(topic_name);
    if (found != registry_.end()) {
        return found->second;
    } else {
        throw std::runtime_error("Topic not found");
    }
}

void switchboard::set_env(const std::string& var, const std::string& val) {
    env_vars_[var] = val;
    setenv(var.c_str(), val.c_str(), 1);
}

std::vector<std::string> switchboard::env_names() const {
    std::vector<std::string> keys(env_vars_.size());
    std::transform(env_vars_.begin(), env_vars_.end(), keys.begin(), [](auto pair) {
        return pair.first;
    });
    return keys;
}

std::string switchboard::get_env(const std::string& var, std::string _default) {
    try {
        if (!env_vars_.at(var).empty())
            return env_vars_.at(var);
        env_vars_.at(var) = _default;
        return _default;
    } catch (std::out_of_range&) {
        char* val = std::getenv(var.c_str());
        if (val) {
            set_env(var, val); // store it locally for faster retrieval
            return {val};
        }
        return _default;
    }
}

bool switchboard::get_env_bool(const std::string& var, const std::string& def) {
    std::string val = get_env(var, def);
    // see if we are dealing with an int value
    try {
        const int i_val = std::stoi(val);
        if (i_val <= 0)
            return false;
        return true;
    } catch (...) { }

    const std::vector<std::string> affirmative{"yes", "y", "true", "on"};
    for (auto s : affirmative) {
        if (std::equal(val.begin(), val.end(), s.begin(), s.end(), [](char a, char b) {
            return std::tolower(a) == std::tolower(b);
        }))
            return true;
    }
    return false;
}

const char* switchboard::get_env_char(const std::string& var, const std::string _default) {
    std::string val = get_env(var, _default);
    if (val.empty())
        return nullptr;
    return strdup(val.c_str());
}

void switchboard::stop() {
    const std::shared_lock lock{registry_lock_};
    for (auto& pair : registry_) {
        pair.second.stop();
    }
}

switchboard::coordinate_system::coordinate_system()
        : position_{0., 0., 0.}
        , orientation_{1., 0., 0., 0.} {
    const char* ini_pose = getenv("WCS_ORIGIN");
    // =
    // if (!ini_pose.empty()) {
    if (ini_pose) {
        std::string        ini_pose_str(ini_pose);
        std::stringstream  iss(ini_pose_str);
        std::string        token;
        std::vector<float> ip;
        while (!iss.eof() && std::getline(iss, token, ',')) {
            ip.emplace_back(std::stof(token));
        }
        if (ip.size() == 3) {
            position_.x() = ip[0];
            position_.y() = ip[1];
            position_.z() = ip[2];
        } else if (ip.size() == 4) {
            orientation_.w() = ip[0];
            orientation_.x() = ip[1];
            orientation_.y() = ip[2];
            orientation_.z() = ip[3];
        } else if (ip.size() == 7) {
            position_.x()    = ip[0];
            position_.y()    = ip[1];
            position_.z()    = ip[2];
            orientation_.w() = ip[3];
            orientation_.x() = ip[4];
            orientation_.y() = ip[5];
            orientation_.z() = ip[6];
        }
    }
}

