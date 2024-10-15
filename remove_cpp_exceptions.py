Import("env")

env.Append(CXXFLAGS=["-fno-exceptions"])
env.Append(LINKFLAGS=["-Wl,--gc-sections"])