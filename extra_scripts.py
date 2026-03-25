Import("env")
import sys, os, shutil

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

# ── Кроссплатформенная загрузка через STM32CubeProgrammer ────
# Windows: ищем в стандартном месте установки.
# Linux/macOS: ожидается, что STM32_Programmer_CLI доступен в PATH
#              (sudo apt install stm32cubeprogrammer  или  brew install stm32cubeprogrammer).
#
# Переменная окружения STM32_PROGRAMMER_PATH позволяет задать путь явно
# на любой ОС: export STM32_PROGRAMMER_PATH=/opt/stm32cubeprog/bin
_cli_env = os.environ.get("STM32_PROGRAMMER_PATH", "")

if _cli_env:
    _cli = os.path.join(_cli_env, "STM32_Programmer_CLI")
elif sys.platform.startswith("win"):
    _win_default = r"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
    _cli = _win_default if os.path.isfile(_win_default) else "STM32_Programmer_CLI"
else:
    # Linux / macOS — должен быть в PATH
    _cli = shutil.which("STM32_Programmer_CLI") or "STM32_Programmer_CLI"

env.Replace(
    UPLOADER=_cli,
    UPLOADCMD=f'"{_cli}" -c port=SWD mode=UR -w "$SOURCE" -v -rst'
)