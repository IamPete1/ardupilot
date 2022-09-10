#pragma once

#include "lua/src/lua.hpp"

// load all known lua bindings into the state
void load_lua_bindings(lua_State *state);

int AP_HAL__I2CDevice_read_registers(lua_State *L);

