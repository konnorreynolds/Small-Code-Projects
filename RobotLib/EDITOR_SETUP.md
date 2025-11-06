# Editor Setup Guide for RobotLib

## Fixing "C++14 extension" Warnings in Clangd/IntelliSense

If you're seeing warnings like:
```
Variable declaration in a constexpr function is a C++14 extension
```

This is because RobotLib uses C++14 features in some `constexpr` functions. The library is **compatible with both C++11 and C++14**, but your editor needs to know which standard to use for analysis.

---

## Quick Fix

**RobotLib includes configuration files:**
- `.clangd` - For clangd users
- `compile_flags.txt` - For clangd fallback
- `.vscode/settings.json` - For VSCode C++ extension
- `.vscode/c_cpp_properties.json` - For VSCode IntelliSense

These tell your editor to use C++14 for code analysis (even though the library compiles with C++11).

**If you still see warnings, try:**
1. Restart your editor/IDE
2. Rebuild the IntelliSense index
3. Check your editor-specific settings below

---

## VSCode Setup

### Using C/C++ Extension (Microsoft)

**Option 1: Use provided config (automatic)**

The `.vscode/` folder is already configured! Just:
1. Open the RobotLib folder in VSCode
2. Press `Ctrl+Shift+P`
3. Type: "C/C++: Edit Configurations (UI)"
4. Verify `C++ standard` is set to `c++14` or higher

**Option 2: Manual setup**

If warnings persist, manually set the standard:

1. Press `Ctrl+Shift+P`
2. Type: "C/C++: Edit Configurations (JSON)"
3. Update `cppStandard`:

```json
{
    "configurations": [
        {
            "name": "Linux",
            "cppStandard": "c++14",
            "includePath": [
                "${workspaceFolder}/include"
            ],
            "defines": [
                "UNITS_CONSTEXPR14=constexpr"
            ]
        }
    ]
}
```

**Option 3: Workspace settings**

Create `.vscode/settings.json` in your project:

```json
{
    "C_Cpp.default.cppStandard": "c++14",
    "C_Cpp.default.includePath": [
        "${workspaceFolder}/lib/RobotLib/include"
    ]
}
```

### Using Clangd Extension

**If using clangd extension instead:**

1. Install the `clangd` extension (disable Microsoft C++ extension)
2. The `.clangd` file will be automatically detected
3. Restart VSCode

**Or add to settings.json:**

```json
{
    "clangd.arguments": [
        "--std=c++14",
        "--header-insertion=never"
    ]
}
```

---

## CLion / IntelliJ Setup

**Method 1: CMake (recommended)**

Add to your `CMakeLists.txt`:

```cmake
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${CMAKE_SOURCE_DIR}/lib/RobotLib/include)
```

CLion will automatically use this.

**Method 2: Project settings**

1. File → Settings → Build, Execution, Deployment → CMake
2. CMake options: `-DCMAKE_CXX_STANDARD=14`

**Method 3: Per-file settings**

1. Right-click on a file → "Override C++ Standard"
2. Select "C++14" or higher

---

## Vim/Neovim with clangd

**Method 1: Use provided .clangd file** (automatic)

The `.clangd` file is already there. Just make sure clangd finds it:

```bash
# Check if clangd detects the config
:CocCommand clangd.userConfig
```

**Method 2: Manual compile_commands.json**

Generate compile commands:

```bash
cd RobotLib
echo '-std=c++14
-I
include' > compile_flags.txt
```

**Method 3: coc-settings.json** (for coc.nvim)

Add to `.vim/coc-settings.json`:

```json
{
    "clangd.arguments": [
        "--std=c++14",
        "--compile-commands-dir=."
    ]
}
```

---

## Emacs with lsp-mode

Add to your Emacs config:

```elisp
(with-eval-after-load 'lsp-mode
  (setq lsp-clients-clangd-args
        '("--std=c++14"
          "--header-insertion=never"
          "--clang-tidy")))
```

Or create `.dir-locals.el` in RobotLib directory:

```elisp
((c++-mode . ((lsp-clients-clangd-args .
              ("--std=c++14")))))
```

---

## Sublime Text with LSP

Install LSP package, then configure `LSP-clangd`:

**Preferences → Package Settings → LSP → Settings**

```json
{
    "clients": {
        "clangd": {
            "command": ["clangd", "--std=c++14"],
            "enabled": true
        }
    }
}
```

---

## Kate / KDevelop

1. Settings → Configure Kate → Plugins → Enable "LSP Client"
2. Settings → LSP Client → User Server Settings

```json
{
    "servers": {
        "cpp": {
            "command": ["clangd", "--std=c++14"],
            "rootIndicationFileNames": [".clangd", "compile_flags.txt"]
        }
    }
}
```

---

## Qt Creator

1. Tools → Options → C++ → Code Model
2. Check "Use Clangd"
3. In "Clangd" tab, add: `--std=c++14` to Additional arguments

Or add to your `.pro` file:

```
CONFIG += c++14
INCLUDEPATH += lib/RobotLib/include
```

---

## Arduino IDE

Arduino IDE doesn't show these warnings (it doesn't have clangd).

**If using external editor with Arduino:**

Make sure your `platformio.ini` or build flags include:

```ini
build_flags =
    -std=c++14
    -I lib/RobotLib/include
```

---

## PlatformIO

Add to `platformio.ini`:

```ini
[env:your_board]
build_flags =
    -std=c++14
    -I lib/RobotLib/include

build_unflags = -std=c++11
```

PlatformIO's IntelliSense will automatically pick this up.

---

## Troubleshooting

### Still seeing warnings after configuration?

**1. Restart the language server:**
- VSCode: `Ctrl+Shift+P` → "Reload Window"
- CLion: File → Invalidate Caches / Restart
- Vim: `:LspRestart`

**2. Rebuild the index:**
- VSCode: `Ctrl+Shift+P` → "C/C++: Reset IntelliSense Database"
- CLion: File → Invalidate Caches → Check "Clear file system cache"

**3. Check clangd is using the right config:**

```bash
# See what flags clangd is using
clangd --check=/path/to/file.cpp
```

**4. Verify your C++ standard:**

Check your compiler:
```bash
g++ --version
g++ -std=c++14 -E -x c++ - -v < /dev/null 2>&1 | grep "c++"
```

---

## Why C++14 for IntelliSense but C++11 Compiles?

**RobotLib uses a compatibility macro:**

```cpp
#if __cplusplus >= 201402L
    #define UNITS_CONSTEXPR14 constexpr
#else
    #define UNITS_CONSTEXPR14
#endif
```

**What this means:**

- **Compiled with C++11:** `UNITS_CONSTEXPR14` → empty, functions become regular functions ✅
- **Compiled with C++14+:** `UNITS_CONSTEXPR14` → `constexpr`, functions are constexpr ✅
- **IntelliSense/Clangd:** Needs to pick one standard for analysis

**Solution:** Tell IntelliSense to use C++14 (the more permissive standard). The code still compiles with C++11!

---

## Summary

**Quick fix checklist:**

- [ ] Use provided `.clangd` and `.vscode/` configs
- [ ] Set editor to C++14 (or C++17, C++20)
- [ ] Add `-std=c++14` to compiler flags
- [ ] Restart editor/LSP server
- [ ] Rebuild IntelliSense index

**The warnings don't affect compilation** - they're just editor hints. But following this guide will make them go away! ✨

---

## Need More Help?

See also:
- [INSTALL.md](INSTALL.md) - Installation instructions
- [PLATFORMIO.md](PLATFORMIO.md) - PlatformIO setup
- [TROUBLESHOOTING.md](TROUBLESHOOTING.md) - Common issues

Or check the clangd documentation: https://clangd.llvm.org/config
