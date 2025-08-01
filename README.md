
# Katana

Katana is a simple BEM code inspired by BYU CCBlade




## Acknowledgements
 - [CCBlade](https://github.com/byuflowlab/CCBlade.jl/tree/master) developed by Prof. Andrew Ning from Brigham Young University (BYU) was used as reference for Katana.
 - The solution method in Katana was adapted from [Ning, Computational Aerodynamics, pp. 200-228](https://byu.box.com/shared/static/ywfayozbj3sr2ot6b32u8nqk5brqvurt.pdf)
 - Other theoretical backing for this code was taken from [Manwell, Wind Energy Explained](https://onlinelibrary.wiley.com/doi/book/10.1002/9781119994367)
 - Thanks to [Prof. John Kurelek](https://smithengineering.queensu.ca/directory/faculty/john-w-kurelek.html) and [Aidan Westdale](https://kurelek.smithengineering.queensu.ca/people/) and many others for their support!
 
 


## Authors

- [Divine Nduka](https://www.linkedin.com/in/divine-nduka-b72479306/?originalSubdomain=ca)


## Appendix

- Currently Katana only works for wind turbines

- Katana can perform analysis of flow conditions where the incoming velocity is > 0, and when the turbine rotational velocity (and thus tip speed ratio) is > 0. No inflow angles less than 0 degrees can be resolved by this code.

- Currently there is no provision for turbine precone angles

- No Mach number corrections are applied in this analysis

- Tip/Hub losses are computed using method outlined by Prandtl (Glauert, H., Airplane propellers, Aerodynamic Theory, Springer, 1935, pp. 169–360.)

- Reynolds number corrections are computed as demonstrated in [Ning, Computational Aerodynamics, pp. 219](https://byu.box.com/shared/static/ywfayozbj3sr2ot6b32u8nqk5brqvurt.pdf)

- The Snell method is applied for rotational corrections to the lift coefficient and Eggers method is used for rotational corrections to the drag coefficient. See [Ning, Computational Aerodynamics, pp. 217 - 218](https://byu.box.com/shared/static/ywfayozbj3sr2ot6b32u8nqk5brqvurt.pdf)


