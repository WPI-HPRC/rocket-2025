{
  description = "C++ platformio flake";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";
    flake-utils = {
      url = "github:numtide/flake-utils";
    };
  };

  outputs = inputs@{ ... }: inputs.flake-utils.lib.eachDefaultSystem (system: 
    let
      pkgs = import inputs.nixpkgs { inherit system; };
    in {
      devShells.default = pkgs.mkShell rec {
        name = "platformio";
        venvDir = "./.venv";
        packages = with pkgs; [
          (platformio.override { platformio-core = platformio-core.overrideAttrs (final: prev: {
            propagatedBuildInputs = prev.propagatedBuildInputs ++ [
              python3Packages.protobuf
              python3Packages.grpcio-tools
            ];
          });})
          clang-tools
          libusb1
        ];

        shellHook = ''
          export LD_LIBRARY_PATH="${pkgs.lib.makeLibraryPath packages}"
        '';
      };
    }
  );
}
