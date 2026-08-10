.PHONY: build launch container connect can-setup sync-orin format hooks

build:
	./scripts/build.sh

launch:
	./scripts/launch.sh

container:
	./scripts/container_launch.sh $(ARGS)

connect:
	./scripts/connect_to_container.sh

can-setup:
	./scripts/setup_can_network.sh

sync-orin:
	./scripts/sync_to_orin.sh $(IP) $(REMOTE_PATH)

format:
	ruff format src
	ruff check --fix src
	shfmt -i 4 -s -w scripts/ .devcontainer/
	npx prettier --write "**/*.{md,json}"

hooks:
	pre-commit install
