using Documenter, BilevelJuMP

makedocs(
    modules = [BilevelJuMP],
    doctest  = false,
    clean    = true,
    format   = Documenter.HTML(mathengine = Documenter.MathJax()),
    sitename = "BilevelJuMP.jl",
    authors  = "Joaquim Garcia",
    pages   = [
        "Home" => "index.md",
        "manual.md",
        "examples.md"
    ]
)

deploydocs(
    repo = "github.com/joaquimg/BilevelJuMP.jl.git",
)