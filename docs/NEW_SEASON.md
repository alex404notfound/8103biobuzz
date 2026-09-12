# 8103 BIOBUZZ season project

This folder is the working copy of the GitHub fork `alex404notfound/8103biobuzz`,
based on the official BIOBUZZ SDK **v12.0**, commit
`e14c2aeb33e84d4ea21697beeea9ac49557be870`.

Our robot source is in `TeamCode/src/main/java`. Pedro, Ivy, FTCLib core,
Dashboard/Panels and Sloth are configured in `TeamCode/build.gradle`; FIRST's
`build.dependencies.gradle` supplies SDK 12.0.0. Build and tune from this folder
using [TUNING.md](TUNING.md).

`origin` is your GitHub fork. `upstream` is FIRST's official SDK. Commit season
changes here and push to `origin`; you do not need to merge this initial import.
The reusable template remains in the separate `8103Template` folder. Changes
between that template and this season project do not synchronize automatically.

The `.8103-template-import.json` file records the original imported file hashes;
season-specific documentation changes happen after that baseline import. Machine
paths in `.ftc-tools.json` are ignored by Git and point to the already installed
`8103Template/.tools` directory. Update `tools_home` if that directory is moved.

For a later season, fork that season's official SDK into a fresh folder, then run
the template's `tools/new_season.py --sdk /path/to/fresh/checkout` with the installed
tools location. Do not rerun the importer over an active season project.
