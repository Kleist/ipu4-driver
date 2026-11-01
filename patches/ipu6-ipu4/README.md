# IPU4 support for the upstream ipu6 driver — patch series

This directory holds the kernel patches that add Intel IPU4 (Broxton /
Apollo Lake, `8086:5a88`) to `drivers/media/pci/intel/ipu6/`. The plan,
the base tree and the PR sequence are in
[`tools/notes/ipu6-ipu4-plan.md`](../../tools/notes/ipu6-ipu4-plan.md).

## Format

- Plain `git format-patch` output, one file per commit, named
  `NNNN-<subject>.patch` so that the shell glob order is the series order.
- Applied with `git am` on top of the pinned tree (`tools/ipu6/pin`) and
  the base series (`patches/ipu6-ipu4p/`, `patches/ipu6-fixes/`) by
  `tools/ipu6/bootstrap.sh` (or `IPU4_FLAVOR=ipu6 tools/bootstrap.sh`).
  Every bootstrap resets the tree to the pin first, so the files here are
  the only source of truth.
- Upstream quality: each patch is what would be mailed to linux-media —
  `media: ipu6: ...` subject, commit message, `Signed-off-by`,
  `scripts/checkpatch.pl --strict` clean, and the tree builds after every
  patch.
- No harness-only code. Test helpers (virt-sensor, QEMU model changes)
  live under `tools/`, never here.
- The patch that adds `8086:5a88` to the PCI ID table
  (`media: ipu6: Enable IPU4 PCI device`) is always the **last** patch,
  as upstream wants the device enabled only once its support is in. The
  series carries it from the start so the tests bind the device the
  normal way; a new patch goes in before it, and regenerating the series
  renumbers it.

## Workflow

```bash
tools/ipu6/bootstrap.sh                   # tools/linux-ipu6 = pin + this series
# ... commit changes in tools/linux-ipu6 ...
git -C tools/linux-ipu6 format-patch -o "$PWD/patches/ipu6-ipu4" series-base
tools/tests/ipu6/ladder.sh                # series-applies, build, checkpatch, ...
```

Regenerating the whole series with `format-patch` keeps the numbering
contiguous; delete the old files first when a patch is dropped or renamed.

## Submitting with b4

[b4](https://b4.docs.kernel.org/) turns a branch into a mailed series,
fills in recipients from `MAINTAINERS`, records the base commit, and
handles review trailers and later revisions. These steps cover the
mainline fixes in `patches/ipu6-fixes/` (ready to send) and, later, this
series. They were walked through with b4 0.16.0 on the 7.4 merge-window
tree, sending into a local SMTP sink (lore and the kernel.org web
endpoint were not reachable from there).

### One-time setup

```bash
pipx install b4              # 0.16 tested; distro packages may be older (0.13 lacks --check/--edit-deps)
git config --global user.name  "Andreas Helbech Kleist"
git config --global user.email "andreaskleist@gmail.com"
# Either SMTP (git send-email settings, e.g. a Gmail app password) ...
git config --global sendemail.smtpServer smtp.gmail.com
git config --global sendemail.smtpServerPort 587
git config --global sendemail.smtpEncryption tls
git config --global sendemail.smtpUser andreaskleist@gmail.com
# ... or kernel.org's web endpoint, no SMTP needed:
git config --global b4.send-endpoint-web https://lkml.kernel.org/_b4_submit
b4 send --web-auth-new       # then: b4 send --web-auth-verify <token from the mail>
```

The web endpoint needs the mails signed (patatt, with a PGP or ed25519
key); with SMTP, signing is optional (`--no-sign`). b4 opens
`$GIT_EDITOR`, then `core.editor`, then `$EDITOR` for the cover letter
and the dependencies.

Work in a mainline clone with history: `get_maintainer.pl` needs it,
and checkpatch (`b4 prep --check`) warns "Unknown commit id" for a
`Fixes:` commit it can't find. The harness trees under `tools/` are
shallow and reset by every bootstrap, so don't use them.

```bash
git clone https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git ~/src/linux
cd ~/src/linux
```

### The mainline fixes (`patches/ipu6-fixes/`)

1. Branch from the newest mainline tag and apply the patches. The empty
   first commit `b4 prep` creates is the cover letter.

   ```bash
   git fetch origin --tags
   b4 prep -n ipu6-fixes -f v7.4-rc1          # or the newest -rc
   git am ~/src/ipu4-driver/patches/ipu6-fixes/*.patch
   ```

2. The patches already carry their `Fixes:` tags, both for 7.4
   merge-window commits by Sakari Ailus:

   - `Fixes: eb68e9f1c6eb ("media: ipu6: Bridge the gap between streams in V4L2 and IPU6 firmware")`
   - `Fixes: 70e3fac3ec78 ("media: ipu6: Move firmware init/cleanup to RPM callbacks")`

   Before sending, check that mainline hasn't fixed either bug
   meanwhile (`git log --oneline v7.4-rc1..origin/master --
   drivers/media/pci/intel/ipu6/`); drop a patch that is no longer
   needed. To find the commit for a new fix, search for the line that
   introduced the bug and add the tag above `Signed-off-by:`:

   ```bash
   git log --oneline -S 'err_requeue_buffers' -- drivers/media/pci/intel/ipu6/ipu6-isys-csi2.c
   git log -1 --abbrev=12 --format='Fixes: %h ("%s")' <sha>
   git rebase -i v7.4-rc1      # mark the patch "edit", then:
   git -c trailer.where=start commit --amend --no-edit --trailer 'Fixes: <sha> ("<subject>")'
   git rebase --continue
   ```

3. Write the cover letter and fill in the recipients:

   ```bash
   b4 prep --edit-cover   # replace the EDITME title, e.g. "media: ipu6: Fix two 7.4 regressions",
                          # and describe each patch in a short paragraph
   b4 prep --auto-to-cc   # To/Cc from MAINTAINERS + get_maintainer.pl
   b4 prep --check        # checkpatch on each patch
   ```

   Look at the recipient list in the cover letter. On the 7.4 tree it
   holds Mauro Carvalho Chehab, Sakari Ailus, Antti Laakso, the Intel
   IPU maintainers, `linux-media@vger.kernel.org` and
   `linux-kernel@vger.kernel.org`.

4. Check the result, send it to yourself, then send it. b4 refuses to
   send while the cover letter still says `EDITME`. Before sending it
   also checks that `--check`, `--auto-to-cc` and (with dependencies)
   `--check-deps` were run on the current version, and if not, asks
   whether to send anyway; then it asks for a final confirmation.

   ```bash
   b4 send -d                   # dry run: prints the mails instead of sending
   b4 send --reflect            # mails the series to you only (To/Cc shown, not delivered)
   b4 send                      # the real thing
   ```

   After the real send, b4 tags the branch `sent/...-v1`, moves it to
   v2 and adds a changelog entry with the v1 link to the cover letter.
   `--reflect` leaves the revision alone.

5. Reviews. Collect trailers (`Reviewed-by:` etc.) from the lore thread
   into the commits, and send a new revision when needed:

   ```bash
   b4 trailers -u           # finds the replies by change-id; asks before rewriting commits
   # ... edit the commits ...
   b4 prep --edit-cover     # fill in the "Changes in v2" entry b4 added
   b4 prep --compare-to v1  # range-diff against what was sent
   b4 send                  # sends [PATCH v2 ...]
   ```

   `b4 trailers -u -m <mbox>` takes the replies from a saved mbox
   instead of lore.

6. Once the fixes are in mainline (or in the media tree that the pin
   follows), delete them from `patches/ipu6-fixes/` and tick the TODO
   in `tools/notes/ipu6-ipu4-plan.md`.

### This series

The series builds on Ruslan Bay's IPU4P series, which is not merged
yet, so it goes out as an RFC on top of it. Put the prerequisites on a
plain branch and fork the b4 branch from it, so only the IPU4 patches
are sent:

```bash
git checkout -b ipu4-base v7.4-rc1
git am ~/src/ipu4-driver/patches/ipu6-ipu4p/*.patch    # Ruslan's series
git am ~/src/ipu4-driver/patches/ipu6-fixes/*.patch    # until mainline has them
b4 prep -n ipu4 -f ipu4-base
git am ~/src/ipu4-driver/patches/ipu6-ipu4/*.patch
b4 prep --set-prefixes RFC   # [PATCH RFC 00/10] ... [PATCH RFC 10/10]
b4 prep --edit-deps          # the prerequisites, see below
b4 prep --check-deps         # fetches them from lore and checks they apply
```

Without dependencies, `base-commit:` in the mails is the top of
`ipu4-base`, which reviewers can't fetch. In `--edit-deps`, list the
prerequisites in order and the real base; at `b4 send` time b4 turns
them into `prerequisite-patch-id:` lines and sets `base-commit:` to the
mainline commit:

```yaml
message-id: <message-id of Ruslan's IPU4P v2 cover letter on lore>
message-id: <message-id of the sent ipu6 fixes, or patch-id: lines>
base-commit: v7.4-rc1
```

`message-id:` entries are resolved through lore. Without lore,
`patch-id:` entries work too, one per prerequisite patch:
`git show <commit> | git patch-id --stable`.

In the cover letter, say that the series applies on top of Ruslan's
IPU4P series (link its lore thread) and the two ipu6 fixes, that it was
tested against this repo's QEMU IPU4 model only, and Cc Ruslan Bay.
Once the IPU4P series is merged, fork from a mainline tag that has it
and drop the RFC prefix (`b4 prep --set-prefixes` with no prefixes).
