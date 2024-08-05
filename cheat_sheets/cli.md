# CLI cheat sheet

## Table of contents

- [CLI cheat sheet](#cli-cheat-sheet)
  - [Table of contents](#table-of-contents)
    - [oh-my-posh](#oh-my-posh)
      - [Custom theme](#custom-theme)
      - [Linux (terminal) setup](#linux-terminal-setup)
      - [Windows (cmd) setup](#windows-cmd-setup)
      - [Windows (PowerShell) setup](#windows-powershell-setup)

### oh-my-posh

[oh-my-posh](https://github.com/JanDeDobbeleer/oh-my-posh/tree/v21.27.0) (v21.27.0 has been used) is a theme engine for any Command Line Interface (CLI).

#### Custom theme

This is an example of a json custom theme file:

```json
{
  "$schema": "https://raw.githubusercontent.com/JanDeDobbeleer/oh-my-posh/main/themes/schema.json",
  "blocks": [
    {
      "alignment": "left",
      "segments": [
        {
          "foreground": "green",
          "properties": {
            "time_format": " 15:04:05 "
          },
          "style": "plain",
          "template": "{{.CurrentDate | date .Format}}",
          "type": "time"
        },
        {
          "foreground": "green",
          "foreground_templates": [
            "{{if (.Staging.Changed)}}#d65905{{end}}",
            "{{if (.Working.Changed)}}#cba406{{end}}"
          ],
          "style": "plain",
          "template": "  <>{</>{{.HEAD}}<>}</>{{if gt .Ahead 0}}<#379595> +{{.Ahead}}</>{{end}}{{if gt .Behind 0}}<#d11a1a> -{{.Behind}}</>{{end}}{{if gt .StashCount 0}}<#9167ab> ({{.StashCount}})</>{{end}}",
          "properties": {
            "fetch_status": true,
            "branch_icon": "",
            "branch_identical_icon": "",
            "branch_ahead_icon": "+",
            "branch_behind_icon": "-",
            "branch_gone_icon": "",
            "commit_icon": "",
            "tag_icon": "",
            "rebase_icon": "REBASING|",
            "cherry_pick_icon": "CHERRY-PICKING|",
            "revert_icon": "REVERTING|",
            "merge_icon": "MERGING|",
            "no_commits_icon": "",
            "gitlab_icon": "",
            "bitbucket_icon": "",
            "azure_devops_icon": "",
            "codecommit_icon": "",
            "codeberg_icon": "",
            "git_icon": ""
          },
          "type": "git"
        }
      ],
      "type": "prompt"
    },
    {
      "alignment": "left",
      "newline": true,
      "segments": [
        {
          "foreground": "blue",
          "properties": {
            "folder_separator_icon": "/",
            "style": "full"
          },
          "style": "plain",
          "template": "{{ .Path }}",
          "type": "path"
        },
        {
          "foreground": "white",
          "style": "plain",
          "template": "$ ",
          "type": "text"
        }
      ],
      "type": "prompt"
    }
  ],
  "version": 2
}
```

#### Linux (terminal) setup

```bash
# Install
curl -s https://ohmyposh.dev/install.sh | bash -s
# Generate autocompletion script
oh-my-posh completion bash >> $HOME/.cache/oh-my-posh/autocompletion.sh
# Place your custom theme file in $HOME/.cache/oh-my-posh/themes
# Apply autocompletion and theme to your terminal by adding the following lines to $HOME/.bashrc
source $HOME/.cache/oh-my-posh/autocompletion.sh
# Point to your corresponding custom theme file
eval "$(oh-my-posh init bash --config $HOME/.cache/oh-my-posh/themes/<CUSTOM_THEME>.omp.json)"
# Restart your terminal
```

#### Windows (cmd) setup

```cmd
REM Install
winget install JanDeDobbeleer.OhMyPosh -s winget
REM Install clink (v1.6.17.1fe17f was used) to be able to apply the theme to cmd
winget install clink
# You can customize clink as follows
clink set autosuggest.hint False
clink set clink.autoupdate off
clink set clink.logo none
clink set history.dupe_mode add
clink set history.shared True
clink set match.ignore_case off
REM Place your custom theme file in %LocalAppData%/Programs/oh-my-posh/themes
REM Apply the theme to cmd
cd %LocalAppData%/clink
vim oh-my-posh.lua
REM Add the following line pointing to your corresponding custom theme file
load(io.popen('oh-my-posh init cmd --config %LocalAppData%/Programs/oh-my-posh/themes/<CUSTOM_THEME>.omp.json'):read("*a"))()
REM Save the file and restart cmd
```

#### Windows (PowerShell) setup

```powershell
# Install
winget install JanDeDobbeleer.OhMyPosh -s winget
# Place you custom theme file in $env:LOCALAPPDATA\Programs\oh-my-posh\themes
# Apply the theme to powershell
vim $PROFILE
# Add the following line pointing to your corresponding custom theme file
oh-my-posh init pwsh --config $env:LOCALAPPDATA\Programs\oh-my-posh\themes\<CUSTOM_THEME>.omp.json  | Invoke-Expression
# Save and restart powershell
```
