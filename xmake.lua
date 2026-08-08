set_project("ik")
set_version("0.1.0")
set_languages("c++17")
add_rules("mode.debug", "mode.release")
add_rules("plugin.compile_commands.autoupdate")

option("with_viewer")
    set_default(false)
    set_showmenu(true)
    set_description("Build SFML viewer target")
option_end()

target("test")
    set_kind("binary")
    set_default(true)
    add_includedirs(".")
    add_files("test.cpp")
    set_warnings("all")
    if is_plat("linux", "macosx") then
        add_syslinks("pthread")
    end

-- Viewer depends on SFML; enable with: xmake f --with_viewer=y
if has_config("with_viewer") then
    add_requires("sfml 2.5.*")
    target("viewer")
        set_kind("binary")
        set_default(false)
        add_includedirs(".")
        add_files("viewer.cpp")
        add_packages("sfml")
end
