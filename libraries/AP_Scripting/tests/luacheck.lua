
-- Don't check globals yet. This requires us to add a list of all the AP bindings
global = false

-- https://luacheck.readthedocs.io/en/stable/warnings.html
ignore = {"631", -- Line is too long
          "611", -- A line consists of nothing but whitespace.
          "612", -- A line contains trailing whitespace.
          "614"} -- Trailing whitespace in a comment.

-- lots of unused variable warnings in the docs
files["libraries/AP_Scripting/docs/docs.lua"] = {ignore = {"212", -- Unused argument.
                                                           "241"}} -- Local variable is mutated but never accessed.

-- Scripting tests taken from lua, should not be changed
files["libraries/AP_Scripting/tests/strings.lua"] = {ignore = {"581"}} -- Negation of a relational operator- operator can be flipped.

files["libraries/AP_Scripting/tests/math.lua"] = {ignore = {"211",  -- Unused local variable.
                                                            "213", -- Unused loop variable.
                                                            "411", -- Redefining a local variable.
                                                            "421", -- Shadowing a local variable.
                                                            "581"}}  -- Negation of a relational operator- operator can be flipped.

-- These lua scripts are not for running on AP
exclude_files = {"Tools/CHDK-Scripts/*", "modules/*"}
