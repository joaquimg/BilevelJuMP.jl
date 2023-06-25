using BilevelJuMP
using Documenter
using Literate
using Test

const _EXAMPLE_DIR = joinpath(@__DIR__, "src", "examples")
const _TUTORIAL_DIR = joinpath(@__DIR__, "src", "tutorials")

"""
    _include_sandbox(filename)
Include the `filename` in a temporary module that acts as a sandbox. (Ensuring
no constants or functions leak into other files.)
"""
function _include_sandbox(filename)
    mod = @eval module $(gensym()) end
    return Base.include(mod, filename)
end

function _file_list(full_dir, relative_dir, extension)
    return map(
        file -> joinpath(relative_dir, file),
        filter(file -> endswith(file, extension), sort(readdir(full_dir))),
    )
end

function link_example(content)
    edit_url = match(r"EditURL = \"(.+?)\"", content)[1]
    footer = match(r"^(---\n\n\*This page was generated using)"m, content)[1]
    content = replace(
        content,
        footer =>
            "!!! info\n    [View this file on Github]($(edit_url)).\n\n" *
            footer,
    )
    return content
end

function literate_directory(dir)
    rm.(_file_list(dir, dir, ".md"))
    for filename in _file_list(dir, dir, ".jl")
        # `include` the file to test it before `#src` lines are removed. It is
        # in a testset to isolate local variables between files.
        @testset "$(filename)" begin
            _include_sandbox(filename)
        end
        Literate.markdown(
            filename,
            dir;
            documenter = true,
            postprocess = link_example,
        )
    end
    return
end

literate_directory(_EXAMPLE_DIR)
literate_directory(_TUTORIAL_DIR)

_REPL_FILES = ["getting_started.md",]
for file in _REPL_FILES
    filename = joinpath(@__DIR__, "src", "tutorials", file)
    content = read(filename, String)
    content = replace(content, "@example" => "@repl")
    write(filename, content)
end

makedocs(;
    modules = [BilevelJuMP],
    doctest = false,
    clean = true,
    format = Documenter.HTML(;
        mathengine = Documenter.MathJax2(),
        prettyurls = get(ENV, "CI", nothing) == "true",
        collapselevel = 1,
    ),
    sitename = "BilevelJuMP.jl",
    authors = "Joaquim Garcia",
    pages = [
        "Home" => "index.md",
        # "Manual" => "manual.md",
        "Tutorials" => joinpath.("tutorials", [
            "getting_started.md",
            "modes.md",
            "lower_duals.md",
            "conic_lower.md",
            "non_linear.md",
            "quad_to_bin.md",
        ]),
        "Examples" => joinpath.("examples", [
            "FOBP_example2.md",
            "FOBP_example3.md",
            "FOBP_example4.md",
            "FOBP_example5.md",
            "DTMP_example1.md",
            "PHTP_example1.md",
            "PHTP_example2.md",
            "SOCBLP_example1.md",
            "MibS_example1.md",
            "MibS_example2.md",
        ]),
        "Background Information" => "background.md",
        "API Reference" => "reference.md",
        "Troubleshooting" => "troubleshooting.md",
    ],
)

deploydocs(;
    repo = "github.com/joaquimg/BilevelJuMP.jl.git",
    push_preview = true,
)
