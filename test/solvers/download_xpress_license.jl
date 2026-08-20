# Copyright (c) 2019: Joaquim Dias Garcia, and contributors
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

# Download the FICO Xpress Community License.
#
# The `xpress` package on PyPI ships the community licence as
# `xpress/license/community-xpauth.xpr` (see https://pypi.org/project/xpress).
# This script pulls just that file out of a wheel, so CI and local runs can
# exercise Xpress without a private licence secret.
#
# The licence caps the solver release it authorises. Xpress_jll is versioned
# after the JLL build rather than the solver, so the two have to be matched by
# hand: `Xpress_jll` v9.8.0 ships FICO release 9.8, which the licence bundled in
# the `xpress` 9.8.1 wheel authorises. A mismatch shows up as
#
#     XpressError(8): ... licensing error 21:
#     Your license only authorizes up to release <X>.
#
# Wheels newer than 9.8.1 no longer bundle a licence at all.
#
# Usage:
#     julia test/solvers/download_xpress_license.jl [output_path]
#
# Defaults to `xpauth.xpr` next to this file. Prints the path it wrote on the
# last line so callers can capture it.
#
# NOTE: never commit the downloaded licence; `*.xpr` is git-ignored.

import Downloads

# Wheel to take the licence from, and the release it authorises. Keep
# XPRESS_JLL_VERSION in test/Project.toml in step with LICENSE_RELEASE.
const WHEEL_VERSION = "9.8.1"
const LICENSE_RELEASE = "9.8"

const LICENSE_IN_WHEEL = "xpress/license/community-xpauth.xpr"

"""
    _wheel_url(version)

Resolve the download URL for an `xpress` wheel via the PyPI JSON API, rather
than guessing the hashed path that files.pythonhosted.org uses.
"""
function _wheel_url(version::AbstractString)
    meta = sprint() do io
        Downloads.download("https://pypi.org/pypi/xpress/$(version)/json", io)
        return
    end
    # Pull out the first wheel URL. Avoid a JSON dependency for one field: the
    # metadata lists each file's "url", and every wheel bundles the same
    # licence, so the first match is fine.
    for m in eachmatch(
        r"\"url\":\s*\"(https://files\.pythonhosted\.org/[^\"]+\.whl)\"",
        meta,
    )
        return String(m.captures[1])
    end
    return error("could not find a wheel URL for xpress $(version) on PyPI")
end

"""
    _extract(zip_path, member, dest)

Extract a single `member` from the zip at `zip_path` to `dest`. Wheels are plain
zip files, and Julia ships p7zip, so no extra package is needed.
"""
function _extract(
    zip_path::AbstractString,
    member::AbstractString,
    dest::AbstractString,
)
    mktempdir() do dir
        # `x` keeps the archive's directory structure, so the member lands at
        # dir/xpress/license/community-xpauth.xpr.
        run(
            pipeline(
                `$(p7zip_exe()) x -y -o$(dir) $(zip_path) $(member)`;
                stdout = devnull,
                stderr = devnull,
            ),
        )
        src = joinpath(
            dir,
            replace(member, '/' => Base.Filesystem.path_separator),
        )
        if !isfile(src)
            error("`$(member)` not found in $(zip_path)")
        end
        cp(src, dest; force = true)
        return
    end
    return dest
end

# Julia ships p7zip; find it wherever this build keeps it.
function p7zip_exe()
    base = joinpath(Sys.BINDIR, "..", "libexec", "julia", "7z")
    for candidate in (base, base * ".exe", Sys.which("7z"), Sys.which("7za"))
        if candidate !== nothing && isfile(candidate)
            return candidate
        end
    end
    return error(
        "could not locate the 7z executable needed to unpack the wheel",
    )
end

function download_license(
    dest::AbstractString = joinpath(@__DIR__, "xpauth.xpr"),
)
    url = _wheel_url(WHEEL_VERSION)
    mktempdir() do dir
        wheel = joinpath(dir, "xpress.whl")
        Downloads.download(url, wheel)
        _extract(wheel, LICENSE_IN_WHEEL, dest)
        return
    end
    text = read(dest, String)
    expiry = match(r"expiry=\"([^\"]+)\"", text)
    release = match(r"fico_xpress_release=\"([^\"]+)\"", text)
    @info "Downloaded Xpress community licence" dest expiry =
        expiry === nothing ? "unknown" : expiry.captures[1] release =
        release === nothing ? "unknown" : release.captures[1]
    if release !== nothing && release.captures[1] != LICENSE_RELEASE
        @warn "Licence release does not match the expected one; Xpress_jll may " *
              "need repinning" got = release.captures[1] expected =
            LICENSE_RELEASE
    end
    return dest
end

if abspath(PROGRAM_FILE) == @__FILE__
    dest = length(ARGS) >= 1 ? ARGS[1] : joinpath(@__DIR__, "xpauth.xpr")
    println(download_license(dest))
end
