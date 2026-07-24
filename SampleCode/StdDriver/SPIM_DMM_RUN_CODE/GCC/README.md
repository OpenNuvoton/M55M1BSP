# NuEclipse GCC Debug Configuration 設定

本範例需在 NuEclipse 的 **Debug Configurations → Startup** 頁面，分別設定 APROM 與 SPIM 映像。

## 設定位置

```text
Run
→ Debug Configurations...
→ GDB Nuvoton Nu-Link Debugging
→ SPIM_DMM_RUN_CODE Release
→ Startup
```

## Load symbols

勾選：

```text
Load symbols
```

並選擇：

```text
Use project binary
SPIM_DMM_RUN_CODE.elf
```

ELF 檔用於載入除錯符號、原始碼行號及函式資訊。

## Load executable to flash

勾選：

```text
Load executable to flash
```

選擇：

```text
Use file
${workspace_loc:/SPIM_DMM_RUN_CODE/Release/SPIM_DMM_RUN_CODE_images/ER_ROM.hex}
```

`Executable offset (hex)` 請留空。

## Load executable to memory

勾選：

```text
Load executable to memory
```

選擇：

```text
Use file
${workspace_loc:/SPIM_DMM_RUN_CODE/Release/SPIM_DMM_RUN_CODE_images/SPIM.hex}
```

`Executable offset (hex)` 請留空。

## Reset 與執行設定

建議設定：

```text
Initial Reset and Halt : Enabled
Reset type             : init
Set breakpoint at      : main
```

## 完成後確認

Debug 前請確認以下檔案已產生：

```text
Release/SPIM_DMM_RUN_CODE.elf
Release/SPIM_DMM_RUN_CODE_images/ER_ROM.hex
Release/SPIM_DMM_RUN_CODE_images/SPIM.hex
```

建議使用右側的 `Workspace...` 按鈕選取 HEX 檔案，避免手動輸入路徑錯誤。
