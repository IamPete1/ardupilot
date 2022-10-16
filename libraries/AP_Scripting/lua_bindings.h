#pragma once

#include "lua/src/lua.hpp"

int lua_millis(lua_State *L);
int lua_micros(lua_State *L);
int lua_mission_receive(lua_State *L);
int AP_Logger_Write(lua_State *L);
int lua_get_i2c_device(lua_State *L);
int AP_HAL__I2CDevice_read_registers(lua_State *L);
int lua_get_CAN_device(lua_State *L);
int lua_get_CAN_device2(lua_State *L);
int lua_userdata_field(lua_State *L, const char *userdata_name, size_t offset, uint8_t type, uint8_t max_array_len, int32_t min_val, int32_t max_val);
