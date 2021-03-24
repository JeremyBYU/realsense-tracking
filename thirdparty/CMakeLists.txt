include(FetchContent)
# Put any LIBRARY dependencies in here
FetchContent_Declare(
  extern_toml11
  GIT_REPOSITORY https://github.com/ToruNiina/toml11.git
  GIT_TAG        v3.3.1
  GIT_PROGRESS   TRUE
)
set(toml11_BUILD_TEST OFF CACHE BOOL "Build toml tests" FORCE)

# FetchContent_Declare(
#   extern_eigen
#   GIT_REPOSITORY https://github.com/eigenteam/eigen-git-mirror.git
#   GIT_TAG        3.3.7
#   GIT_PROGRESS   TRUE
# )

FetchContent_MakeAvailable(extern_toml11)
# FetchContent_MakeAvailable(extern_eigen)