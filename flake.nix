{
  description = "Flake for mc-force-shoe-plugin with a mc-rtc-superbuild shell";

  inputs = {
    mc-rtc-nix.url = "github:mc-rtc/nixpkgs";
    flake-parts.follows = "mc-rtc-nix/flake-parts";
    systems.follows = "mc-rtc-nix/systems";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, ... }:
      {
        systems = import inputs.systems;
        imports = [
          inputs.mc-rtc-nix.flakeModule
          {
            flakoboros = {
              extraPackages = [ "ninja" ];

              overrideAttrs.mc-force-shoe-plugin = {
                src = lib.cleanSource ./.;
              };

              # Define a custom superbuild configuration
              # This will make all
              overrides.mc-rtc-superbuild-minimal =
                { pkgs-final, pkgs-prev, ... }:
                let
                  cfg-prev = pkgs-prev.mc-rtc-superbuild-minimal.superbuildArgs;
                in
                {
                  superbuildArgs = cfg-prev // {
                    pname = "mc-force-shoe-plugin-superbuild";
                    configs = [ "${pkgs-final.mc-force-shoe-plugin}/lib/mc_plugins/etc/mc_rtc.yaml" ];
                    plugins = [ pkgs-final.mc-force-shoe-plugin ];
                    apps = [ pkgs-final.mc-rtc-magnum ];
                  };
                };

            };
          }
        ];
        perSystem =
          { pkgs, ... }:
          {
            # define a devShell called local-superbuild with the superbuild configuration above
            # you can also override attributes to add additional shell functionality
            packages.default = pkgs.mc-rtc-superbuild-minimal;
            devShells.default =
              (pkgs.callPackage "${inputs.mc-rtc-nix}/shell.nix" {
                mc-rtc-superbuild = pkgs.mc-rtc-superbuild-minimal;
              }).overrideAttrs
                (old: {
                  shellHook = ''
                    ${old.shellHook or ""}

                    echo ""
                    echo "Welcome to ${pkgs.mc-rtc-superbuild-minimal.superbuildArgs.pname} !"
                    echo "Run:"
                    echo "$ mc-rtc-magnum & # to display the gui"
                    echo "$ mc_rtc_ticker # to test the plugin's default configuration from \$MC_RTC_CONTROLLER_CONFIG"
                    echo "----"
                  '';
                });
          };
      }
    );
}
