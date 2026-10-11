#!/usr/bin/env bash
# Move tools/ipu6/pin to the newest mainline tag (release or -rc) at
# IPU6_LINUX_URL. Run weekly by
# .github/workflows/bump-kernel-pins.yml; the bump PR's vm-ipu6 run shows
# whether the series still apply.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
PIN="$HERE/pin"
# shellcheck source=pin
. "$PIN"

refs="$(git -c versionsort.suffix=-rc ls-remote --tags --sort=-v:refname \
	"$IPU6_LINUX_URL" 'v*')"
tag="$(sed -nE '/\trefs\/tags\/v[0-9]+\.[0-9]+(-rc[0-9]+)?$/{s@.*/@@p;q}' \
	<<< "$refs")"
sha="$(awk -v r="refs/tags/$tag^{}" '$2 == r { print $1 }' <<< "$refs")"
if [[ -z "$tag" || -z "$sha" ]]; then
	echo "::error::no mainline tag found at $IPU6_LINUX_URL" >&2
	exit 1
fi

if [[ "$tag" == "$IPU6_LINUX_TAG" ]]; then
	echo "tools/ipu6/pin $tag (unchanged)"
	exit 0
fi
sed -i -E \
	-e "s@^IPU6_LINUX_TAG=.*@IPU6_LINUX_TAG=\"$tag\"@" \
	-e "s@(IPU4_IPU6_LINUX_SHA:-)[0-9a-f]+@\1$sha@" "$PIN"
echo "tools/ipu6/pin $IPU6_LINUX_TAG -> $tag ($sha)"
