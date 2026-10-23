{
  description = "slopvr: PlayStation VR2 driver for SteamVR on Linux (pure C)";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { self, nixpkgs }:
    let
      system = "x86_64-linux";

      # The packages, as a function of a nixpkgs so the NixOS module builds them with the system's own nixpkgs.
      # Everything is ./build.sh; see there.
      mkPackages = pkgs: rec {
        src = pkgs.lib.fileset.toSource { root = ./.; fileset = pkgs.lib.fileset.unions [ ./src ./resources ./build.sh ]; };

        # the SteamVR driver (release build): driver dir, Vulkan layer, udev rules
        slopvr = pkgs.stdenv.mkDerivation {
          pname = "slopvr"; version = "0.1.0"; inherit src;
          nativeBuildInputs = [ pkgs.binutils ];
          dontConfigure = true;
          buildPhase = "bash ./build.sh driver";
          installPhase = ''
            mkdir -p $out/lib/udev/rules.d $out/share/vulkan/implicit_layer.d
            cp -r build/slopvr $out/slopvr
            cp build/libslopvr_vk.so $out/lib/
            cp build/70-slopvr.rules $out/lib/udev/rules.d/
            sed "s|\"library_path\":\"[^\"]*\"|\"library_path\":\"$out/lib/libslopvr_vk.so\"|" build/slopvr_vk.json > $out/share/vulkan/implicit_layer.d/slopvr_vk.json
          '';
          meta = { description = "PlayStation VR2 driver for SteamVR on Linux"; homepage = "https://github.com/egbaydarov/slopvr"; platforms = [ system ]; };
        };

        # the camera view / tracking debugger / calibration flow (debug build); quit SteamVR before running it
        psvr2linuxtest = pkgs.stdenv.mkDerivation {
          pname = "psvr2linuxtest"; version = "0.1.0"; inherit src;
          nativeBuildInputs = [ pkgs.pkg-config ];
          buildInputs = [ pkgs.SDL2 pkgs.wayland pkgs.libdrm ];
          dontConfigure = true;
          buildPhase = "bash ./build.sh test";
          installPhase = ''
            mkdir -p $out/bin $out/share/slopvr
            cp build/psvr2linuxtest $out/bin/
            cp -r resources build/70-slopvr.rules $out/share/slopvr/
          '';
          meta = { description = "PlayStation VR2 tracking debugger and calibration tool for slopvr"; mainProgram = "psvr2linuxtest"; platforms = [ system ]; };
        };

        # slopvr-install: register a driver build with the calling user's SteamVR (./build.sh register)
        mkInstall = drv: pkgs.writeShellApplication {
          name = "slopvr-install";
          runtimeInputs = [ pkgs.coreutils pkgs.gnugrep pkgs.jq pkgs.procps ];
          text = ''SLOPVR_DIR=${drv}/slopvr exec bash ${src}/build.sh register'';
        };
        install = mkInstall slopvr;

        default = slopvr;
      };

      pkgs = nixpkgs.legacyPackages.${system};
      slop = mkPackages pkgs;
    in {
      packages.${system} = { inherit (slop) slopvr psvr2linuxtest install default; };

      apps.${system} = {
        install = { type = "app"; program = "${slop.install}/bin/slopvr-install"; };
        test = { type = "app"; program = "${slop.psvr2linuxtest}/bin/psvr2linuxtest"; };
        default = self.apps.${system}.install;
      };

      # everything ./build.sh needs: a C compiler, binutils for the glibc check, SDL2 / wayland / libdrm for the test tool
      devShells.${system}.default = pkgs.mkShell {
        packages = [ pkgs.binutils pkgs.jq pkgs.usbutils pkgs.pkg-config pkgs.SDL2 pkgs.wayland pkgs.libdrm ];
        shellHook = "echo 'slopvr dev shell: ./build.sh | ./build.sh test | ./build.sh install'";
      };

      # NixOS module.  configuration.nix (classic):
      #   imports = [ (builtins.getFlake "github:egbaydarov/slopvr").nixosModules.default ];
      #   services.slopvr.enable = true;
      # or with flakes: input github:egbaydarov/slopvr, module slopvr.nixosModules.default.
      # Gives: udev rules for the headset and the controllers, Bluetooth with the setting the Sense controllers need,
      # slopvr-install and the test tool in PATH, and a user service that registers the driver with SteamVR at login
      # (the registration lives in the user's home, SteamVR keeps it there).
      nixosModules.default = { config, lib, pkgs, ... }:
        let cfg = config.services.slopvr; p = mkPackages pkgs; inst = p.mkInstall cfg.package; in {
          options.services.slopvr = {
            enable = lib.mkEnableOption "slopvr, the PlayStation VR2 driver for SteamVR";
            package = lib.mkOption { type = lib.types.package; default = p.slopvr; description = "The driver package."; };
            testTool = lib.mkOption { type = lib.types.bool; default = true; description = "Install psvr2linuxtest."; };
            registerAtLogin = lib.mkOption { type = lib.types.bool; default = true; description = "Run slopvr-install for every user at login, so a rebuild needs no manual step."; };
            bluetooth = lib.mkOption { type = lib.types.bool; default = true; description = "Enable Bluetooth and let BlueZ accept HID connections from paired but not bonded devices (a Sense controller in that state gets no hidraw device otherwise)."; };
          };
          config = lib.mkIf cfg.enable {
            services.udev.packages = [ cfg.package ];
            environment.systemPackages = [ inst ] ++ lib.optional cfg.testTool p.psvr2linuxtest;
            hardware.bluetooth = lib.mkIf cfg.bluetooth { enable = lib.mkDefault true; input.General.ClassicBondedOnly = lib.mkDefault false; };
            systemd.user.services.slopvr-register = lib.mkIf cfg.registerAtLogin {
              description = "Register the slopvr driver with this user's SteamVR";
              wantedBy = [ "default.target" ];
              serviceConfig = { Type = "oneshot"; ExecStart = "${inst}/bin/slopvr-install"; };
            };
          };
        };
    };
}
