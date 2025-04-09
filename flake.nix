
{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/develop";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";  # IMPORTANT!!!
    carla = { url = "github:CTU-IIG/carla-simulator.nix"; inputs.nixpkgs.follows = "nix-ros-overlay/nixpkgs"; };
  };
  outputs = { self, nix-ros-overlay, nixpkgs, carla }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        applyDistroOverlay =
          rosOverlay: rosPackages:
          rosPackages
          // builtins.mapAttrs (
            rosDistro: rosPkgs: if rosPkgs ? overrideScope then rosPkgs.overrideScope rosOverlay else rosPkgs
          ) rosPackages;
        rosDistroOverlays = final: prev: {
          # Apply the overlay to multiple ROS distributions
          rosPackages = applyDistroOverlay (import ./overlay.nix) prev.rosPackages;
        };
        pkgs = import nixpkgs {
          inherit system;
          overlays = [
            nix-ros-overlay.overlays.default
            rosDistroOverlays
            carla.overlays."0.9.15"
          ];
        };
        rosDistro = "jazzy";
      in {
        apps.default = {
          type = "app";
          meta.description = "Run carla_camera_publisher compiled against ROS ${rosDistro}.";
          program = let
            wrapper = pkgs.writeShellApplication {
              name = "carla-camera-publisher-wrapper";
              runtimeInputs = [
                (with pkgs.rosPackages.${rosDistro}; buildEnv {
                  paths = [
                    compressed-image-transport
                    ffmpeg-image-transport
                    foxglove-compressed-video-transport
                    ros2run
                    self.packages.${system}.default
                  ];
                })
              ];
              text = ''ros2 run carla_camera_publisher carla_camera_publisher "$@"'';
            };
          in
            "${wrapper}/bin/carla-camera-publisher-wrapper";
        };
        packages = builtins.intersectAttrs (import ./overlay.nix null null) pkgs.rosPackages.${rosDistro} // {
          default = self.packages.${system}.carla-camera-publisher;
          dump-parameters = pkgs.writeShellApplication {
            name = "dump-parameters";
            runtimeInputs = with pkgs; [
              rosPackages.${rosDistro}.ros-core
              yq-go
            ];
            text = ''
              set -x
              (
                  ${self.packages.${system}.default}/lib/carla_camera_publisher/carla_camera_publisher > /dev/null &
                  ros2 param dump --no-daemon /carla_camera_publisher
                  kill $!
                  wait
              ) | yq eval '.["/carla_camera_publisher"].ros__parameters|omit(["carla_camera_publisher", "ffmpeg_image_transport", "qos_overrides", "use_sim_time"])'
            '';
          };
        };
        checks = builtins.intersectAttrs (import ./overlay.nix null null) pkgs.rosPackages.${rosDistro};
        devShells.default = pkgs.mkShell {
          name = "CARLA camera publisher";
          inputsFrom = [ self.packages.${system}.carla-camera-publisher ];
          packages = [
            pkgs.colcon
            # ... other non-ROS packages
            (with pkgs.rosPackages.${rosDistro}; buildEnv {
              paths = [
                compressed-image-transport
                ffmpeg-image-transport
                foxglove-compressed-video-transport
                ros-core
              ];
            })
          ];
        };
      });
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
