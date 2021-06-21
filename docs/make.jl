using Documenter, BilevelJuMP, DocumenterCitations

bib = CitationBibliography(joinpath(@__DIR__, "src/references.bib"), sorting = :nyt)

makedocs(
    bib,
    modules = [BilevelJuMP],
    doctest  = false,
    clean    = true,
    format   = Documenter.HTML(mathengine = Documenter.MathJax2()),
    sitename = "BilevelJuMP.jl",
    authors  = "Joaquim Garcia",
    pages   = [
        "Home" => "index.md",
        "manual.md",
        "theory.md"
    ]
)

deploydocs(
    repo = "github.com/joaquimg/BilevelJuMP.jl.git",
    push_preview = true
)