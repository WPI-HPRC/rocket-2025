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
      devShells.default = pkgs.mkShell {
        name = "platformio";
        packages = with pkgs; [
          platformio
          clang-tools
        ];
      };
    }
  );
}
