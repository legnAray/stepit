# policy_neuro_redis

StepIt plugin for reading vector fields from Redis and feeding them into the StepIt neuro policy.

### Prerequisites

Install [hiredis](https://github.com/redis/hiredis.git) and [nlohmann-json3](https://github.com/nlohmann/json) via `apt`:

```shell
sudo apt install libhiredis-dev nlohmann-json3-dev
```

### Provided Factories

`stepit::neuro_policy::Module`:

- `redis_field_subscriber`: reads configured Redis keys or hash fields, parses JSON arrays of numbers into float vectors, and provides the named StepIt fields declared in its config map.
