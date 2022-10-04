using BilevelJuMP
using Documenter
using Literate
using Test

const _EXAMPLE_DIR = joinpath(@__DIR__, "src", "examples")

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
        footer => "!!! info\n    [View this file on Github]($(edit_url)).\n\n" * footer,
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
        Literate.markdown(filename, dir; documenter = true, postprocess = link_example)
    end
    return
end

literate_directory(_EXAMPLE_DIR)

makedocs(
    modules = [BilevelJuMP],
    doctest = false,
    clean = true,
    format = Documenter.HTML(
        mathengine = Documenter.MathJax2(),
        prettyurls = get(ENV, "CI", nothing) == "true",
    ),
    sitename = "BilevelJuMP.jl",
    authors = "Joaquim Garcia",
    pages = [
        "Home" => "index.md",
        "Manual" => "manual.md",
        "Examples" =>
            [joinpath("examples", f) for f in readdir(_EXAMPLE_DIR) if endswith(f, ".md")],
    ],
)

deploydocs(repo = "github.com/joaquimg/BilevelJuMP.jl.git", push_preview = true)
