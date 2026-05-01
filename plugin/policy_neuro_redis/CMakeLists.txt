cmake_minimum_required(VERSION 3.16)
project(stepit_plugin_policy_neuro_redis)

stepit_add_plugin(${PROJECT_NAME}
    SOURCES
      src/field_subscriber_redis.cpp
    INCLUDES
      include
    DEPENDS
      policy_neuro
      redis_base
)
