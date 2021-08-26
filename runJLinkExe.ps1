
param (
    [string]$JLinkPath,
    [string]$JLinkGDBServerPath,
    [string]$JLinkRTTClientPath
)

& "${JLinkPath}" -device stm32f407ig -if swd -speed 4000 -autoconnect 1 -RTTTelnetPort 19022