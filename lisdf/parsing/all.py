from typing import Tuple

import lisdf.components as C
from lisdf.parsing.pddl_j import load_pddl
from lisdf.parsing.sdf_j import load_sdf


def load_all(
    lisdf_file: str, domain_file: str, problem_file: str, *, verbose: bool = False
) -> Tuple[C.LISDF, C.PDDLDomain, C.PDDLProblem]:
    lisdf = load_sdf(lisdf_file, verbose=verbose)
    domain, problem = load_pddl(domain_file, problem_file, lisdf=lisdf)
    return lisdf, domain, problem
