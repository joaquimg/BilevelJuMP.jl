using Documenter, BilevelJuMP

makedocs(
    modules = [BilevelJuMP],
    doctest  = false,
    clean    = true,
    format   = Documenter.HTML(mathengine = Documenter.MathJax2()),
    sitename = "BilevelJuMP.jl",
    authors  = "Joaquim Garcia",
    pages   = [
        "Home" => "index.md",
        "manual.md"
    ]
)

deploydocs(
    repo = "github.com/joaquimg/BilevelJuMP.jl.git",
    push_preview = true
)