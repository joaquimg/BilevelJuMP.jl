# [API](@id API)

This section documents the BilevelJuMP API.

As a JuMP extension, most JuMP functions should just work.
Some JuMP function will return error saying they are not
implemented for BileveJuMP structures such as `BilevelModel`.
If that happens and you consider that function should be implemented,
please, open an issue.

## Constructors

```@docs
BilevelModel
Upper
Lower
DualOf
```

### Advanced constructors

```@docs
UpperOnly
LowerOnly
```

## Enums

```@docs
BilevelJuMP.Level
BilevelJuMP.LOWER_BOTH
BilevelJuMP.UPPER_BOTH
BilevelJuMP.LOWER_ONLY
BilevelJuMP.UPPER_ONLY
BilevelJuMP.DUAL_OF_LOWER
```

```@docs
BilevelJuMP.IndicatorSetting
BilevelJuMP.ZERO_ONE
BilevelJuMP.ZERO_ZERO
BilevelJuMP.ONE_ONE
```

## Structs

```@docs
BilevelVariableRef
BilevelAffExpr
BilevelQuadExpr
```

## Modes

```@docs
BilevelJuMP.SOS1Mode
BilevelJuMP.FortunyAmatMcCarlMode
BilevelJuMP.IndicatorMode
BilevelJuMP.ProductMode
BilevelJuMP.StrongDualityMode
BilevelJuMP.ComplementMode
BilevelJuMP.MixedMode
```

## Bound hints

```@docs
BilevelJuMP.set_dual_upper_bound_hint
BilevelJuMP.get_dual_upper_bound_hint
BilevelJuMP.set_dual_lower_bound_hint
BilevelJuMP.get_dual_lower_bound_hint
BilevelJuMP.set_primal_upper_bound_hint
BilevelJuMP.get_primal_upper_bound_hint
BilevelJuMP.set_primal_lower_bound_hint
BilevelJuMP.get_primal_lower_bound_hint
```

## Attributes getters and setters

```@docs
BilevelJuMP.lower_objective_value
BilevelJuMP.build_time
BilevelJuMP.set_mode
BilevelJuMP.get_mode
BilevelJuMP.unset_mode
BilevelJuMP.set_copy_names
BilevelJuMP.get_copy_names
BilevelJuMP.unset_copy_names
BilevelJuMP.set_pass_start
BilevelJuMP.get_pass_start
BilevelJuMP.unset_pass_start
```