Import("env")

# ── Патч флагов FPU ─────────────────────────────────────────
# PlatformIO для stm32cube иногда добавляет softfp/vfpv3-d16.
# STM32F407 имеет FPUv4-SP — заменяем на hard ABI.
REMOVE = {"-mfloat-abi=softfp", "-mfloat-abi=soft", "-mfpu=vfpv3-d16"}
ADD    = ["-mcpu=cortex-m4", "-mthumb", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"]

for k in ("CCFLAGS", "CXXFLAGS", "ASFLAGS", "LINKFLAGS"):
    env[k] = [f for f in env.get(k, []) if f not in REMOVE]
    for flag in ADD:
        if flag not in env[k]:
            env[k].append(flag)

# ── Линковка libmicroros.a ───────────────────────────────────
# Статическая библиотека micro-ROS собрана под STM32F407 (Humble).
# Пересборка нужна только при смене ROS-пакетов — см. README.
env.Append(
    LIBS=["microros"],
    LIBPATH=[
        "lib/micro_ros_platformio/libmicroros"
    ],
    CPPPATH=[
        "lib/micro_ros_platformio/libmicroros/include"
    ]
)