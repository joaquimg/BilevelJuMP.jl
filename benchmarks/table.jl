using DataFrames
using CSV
using Printf


cd(@__DIR__)

FILE = ".\\bilevel_results.csv"
df = DataFrame(CSV.File(FILE))

function mystr(t::Number, g::Number, opt = "cbc")
    if g < 1e-4
        t = min(t, 600)
        return "$(trunc.(Int,t))s"
    elseif g < 10
        return "$(trunc.(Int,100*g))\\%"
    elseif opt in ["ipopt", "knitro"]
        t = min(t, 600)
        return "$(trunc.(Int,t))s"
    else
        @show g, t
        return " - "
    end
end
# mystr(t::String, g::AbstractString, opt::String = "cbc") = @show g, t;""
mystr(t, g, opt = "cbc") = ""

df2 = transform(df, [:solve_time, :gap] => (t,g) -> mystr.(t, g))

function int_or_empty(str)
    return strip(str)
    if length(str) > 0
        return "$(parse(Int, str))"
    else
        return ""
    end
end

fnd(df, sym, str) = strip.(df[!,sym]) .== str
fnd(df, sym, str, str2) = strip.(df[!,sym]) .== "($str;$str2)"

mrow(NAME) = "\\parbox[t]{2mm}{\\multirow{3}{*}{\\rotatebox[origin=c]{90}{$(NAME)}}}"

function ptable(df, modes)

instances = Set()
for i in eachrow(df)
    s = string(i[:inst])
    s = replace(s, "("=>"")
    s = replace(s, ")"=>"")
    s = split(s,';')
    push!(instances, (strip(i[:prob]), int_or_empty(s[2]), int_or_empty(s[1])))
end
# ordered
@show instances = sort!(collect(instances))#, order = Base.Reverse)

solvers = Set()
for i in eachrow(df), mode in modes
    (_opt, _mode) = split(strip(i[:opt_mode]), "_")
    if mode == _mode
        push!(solvers, _opt)
    end
end
# ordered
@show solvers = sort!(collect(solvers))#, order = Base.Reverse)

s = ""

s *= "\\begin{table}[!ht]\n"
s *= "\\resizebox{\\textwidth}{!}{\n"
s *= "\\begin{tabular}{rr$("|rr$(true ? "r" : "")"^(length(solvers)))}\n"

lp = "NULL"

c = 0
s *= "\\toprule\n"

# first two columns are case data
s *= " & "
for opt in solvers
    s *= " & \\multicolumn{3}{c}{$opt} $(opt != solvers[end] ? "\\vline" : "")"
end
s *= " \\\\\n"

s *= " & Inst"
for opt in solvers
    s *= " & Obj & Gap " #(\\%)
    if true
        s *= " & Time "
    end

end
s *= " \\\\\n"

if false
s *= " & "
for opt in solvers
    s *= " &   & time(s) "
end
s *= " \\\\\n"
end



for (p,i1,i2) in instances
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
            @show opt_mode = "$(opt)_$(mode)"
            @show i2, i1
            o = ""
            g = ""
            t = ""
            o = try
                minimum(df[fnd(df, :opt_mode, opt_mode) .& fnd(df, :prob, p) .& fnd(df, :inst, i2, i1), :upper_obj])
            catch
                ""
            end
            g = try
                minimum(df[fnd(df, :opt_mode, opt_mode) .& fnd(df, :prob, p) .& fnd(df, :inst, i2, i1), :gap])
            catch
                ""
            end
            t = try
                minimum(df[fnd(df, :opt_mode, opt_mode) .& fnd(df, :prob, p) .& fnd(df, :inst, i2, i1), :solve_time])
            catch
                ""
            end
            v = mystr(t, g, opt)
            @show t, g, v, o
            s *= " & "
            s *= pobj(o)
            s *= " & "
            if true
                s *= pgap(g)
                s *= " & "
                s *= ptime(t)
            else
                if length(v) > 0
                    s *= v
                end
            end
        end
    end
    s *= " \\\\\n"
end
s *= "\\bottomrule\n"
s *= "\\end{tabular}\n"
s *= "}\n"
s *= "\\caption{$(modes[1]), Time in seconds (s), Gap in percent (\\%).}\n"
s *= "\\label{table_$(modes[1])}\n"
s *= "\\end{table}\n"


@show modes[1]
println(s)

f = open("table2_$(modes[1]).tex", "w")
print(f, s)
close(f)

@show pwd()

end

function pgap(g::Number)
    if g < 1e-4
        return "0"
    elseif g < 10
        return "$(trunc.(Int,100*g))"
    else
        return " - "
    end
end
function ptime(t::Number)
    if t < 600 - 5
        return "$(trunc.(Int,t))"
    else
        return " - "
    end
end
pobj(o::String) = " - "
pobj(o::Number) = abs(o) < 1e8 ? Printf.@sprintf("%4.2f", o) : " - "



modes = [
    # ["fa100"],
    # ["sos1"],
    # ["indc"],
    # ["prod100"],
    # ["sd100"],
    ["prod"],
    ["sd"],
    ]
for mode in modes
    ptable(df, mode)
end

# CSV.write(FILE2, df2)