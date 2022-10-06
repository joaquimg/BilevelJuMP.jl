using DataFrames
using CSV
using Printf

cd(@__DIR__)

FILE = "C:\\Users\\joaquimgarcia\\Desktop\\bilevel_results.csv"
FILE2 = "C:\\Users\\joaquimgarcia\\Desktop\\bilevel_results2.csv"
df = DataFrame(CSV.File(FILE))

function mystr(t::Number, g::Number, opt::String = "cbc")
    if g < 1e-4
        t = min(t, 600)
        return "$(trunc.(Int,t))s"
    elseif g < 10
        return "$(trunc.(Int,100*g))\\%"
    elseif opt in ["ipopt", "knitro"]
        t = min(t, 600)
        return "$(trunc.(Int,t))s"
    else
        return ""
    end
end
mystr(t::String, g::String, opt::String = cbc) = ""

df2 = transform(df, [:time, :gap] => (t, g) -> mystr.(t, g))

function int_or_empty(str)
    return strip(str)
    if length(str) > 0
        return "$(parse(Int, str))"
    else
        return ""
    end
end

fnd(df, sym, str) = strip.(df[sym]) .== str
fnd(df, sym, str, str2) = strip.(df[sym]) .== "($str;$str2)"

function mrow(NAME)
    return "\\parbox[t]{2mm}{\\multirow{3}{*}{\\rotatebox[origin=c]{90}{$(NAME)}}}"
end

function ptable(df, modes)
    instances = Set()
    for i in eachrow(df)
        s = string(i[:inst])
        s = replace(s, "(" => "")
        s = replace(s, ")" => "")
        s = split(s, ';')
        push!(
            instances,
            (strip(i[:prob]), int_or_empty(s[1]), int_or_empty(s[2])),
        )
    end
    # ordered
    @show instances = sort!(collect(instances))#, order = Base.Reverse)

    solvers = Set()
    for i in eachrow(df), mode in modes
        if mode == strip(i[:mode])
            push!(solvers, i[:opt])
        end
    end
    # ordered
    @show solvers = sort!(collect(solvers))#, order = Base.Reverse)

    s = ""

    s *= "\\begin{table}[]\n"
    s *= "\\resizebox{\\textwidth}{!}{\\%\n"
    s *= "\\begin{tabular}{llllllllllllllll}\n"

    lp = "NULL"

    c = 0
    s *= "\\toprule\n"

    # first two columns are case data
    s *= " & "
    for opt in solvers
        s *= " & \\multicolumn{2}{c}{$opt}"
    end
    s *= " \\\\\n"

    s *= " & inst"
    for opt in solvers
        s *= " & obj & gap(\\%) "
    end
    s *= " \\\\\n"

    s *= " & "
    for opt in solvers
        s *= " &   & time(s) "
    end
    s *= " \\\\\n"

    for (p, i1, i2) in instances
        if lp == p
            c += 1
        else
            c = 0
            s *= "\\midrule\n"
        end
        lp = p
        if c == 2
            s *= mrow(p)
        end
        if length(i2) > 0
            s *= " & $i1/$i2 "
        else
            s *= " & $i1 "
        end
        for opt in solvers
            for mode in modes
                o = ""
                g = ""
                t = ""

                o = try
                    minimum(
                        df[
                            fnd(
                                df,
                                :mode,
                                mode,
                            ).&fnd(
                                df,
                                :opt,
                                opt,
                            ).&fnd(df, :prob, p).&fnd(df, :inst, i1, i2),
                            :obj,
                        ],
                    )
                catch
                    ""
                end
                g = try
                    minimum(
                        df[
                            fnd(
                                df,
                                :mode,
                                mode,
                            ).&fnd(
                                df,
                                :opt,
                                opt,
                            ).&fnd(df, :prob, p).&fnd(df, :inst, i1, i2),
                            :gap,
                        ],
                    )
                catch
                    ""
                end
                t = try
                    minimum(
                        df[
                            fnd(
                                df,
                                :mode,
                                mode,
                            ).&fnd(
                                df,
                                :opt,
                                opt,
                            ).&fnd(df, :prob, p).&fnd(df, :inst, i1, i2),
                            :time,
                        ],
                    )
                catch
                    ""
                end
                # @show t, g
                v = mystr(t, g, opt)
                s *= " & "
                s *= pobj(o)
                s *= " & "
                if length(v) > 0
                    s *= v
                end
            end
        end
        s *= " \\\\\n"
    end
    s *= "\\bottomrule\n"

    s *= "\\end{tabular}\n"
    s *= "}\n"
    s *= "\\end{table}\n"

    println(s)

    f = open("table_$(modes[1]).tex", "w")
    print(f, s)
    close(f)

    @show pwd()
end

pobj(o::String) = ""
pobj(o::Number) = abs(o) < 1e8 ? Printf.@sprintf("%4.2f", o) : ""

modes = [
    ["fa10"],
    ["fa100"],
    ["sos1"],
    ["prod10"],
    ["prod100"],
    ["sd10"],
    ["sd100"],
    ["prod", "sd"],
]
for mode in modes
    ptable(df, mode)
end

# CSV.write(FILE2, df2)
