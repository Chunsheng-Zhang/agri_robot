"""Small bounded real-coded GA with copied elites and seeded randomness."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class Individual:
    genes: tuple
    cost: float


def minimize(evaluate, bounds, *, initial=(), population_size=12, generations=7,
             seed=27, on_generation=None):
    """Minimize a deterministic scalar cost; full evaluation precedes selection.

    Two immutable elites, tournament selection, blend crossover, bounded
    Gaussian mutation and one random immigrant per later generation.
    """
    limits=np.asarray(bounds,dtype=float)
    if limits.ndim!=2 or limits.shape[1]!=2 or not np.isfinite(limits).all() or np.any(limits[:,1]<=limits[:,0]):
        raise ValueError('Bounds must be finite increasing pairs')
    if population_size<6 or generations<1:raise ValueError('Need population>=6 and generations>=1')
    low,high=limits.T;span=high-low;rng=np.random.default_rng(seed)
    population=[]
    for genes in initial:
        g=np.asarray(genes,dtype=float)
        if g.shape!=low.shape or not np.isfinite(g).all() or np.any(g<low) or np.any(g>high):
            raise ValueError('Initial genes outside bounds')
        population.append(g.copy())
    if len(population)>population_size:raise ValueError('Too many initial candidates')
    population.extend(rng.uniform(low,high) for _ in range(population_size-len(population)))
    best=None;history=[]
    for generation in range(generations):
        evaluated=[]
        for genes in population:
            frozen=tuple(float(x) for x in genes)
            cost=float(evaluate(frozen))
            if not math.isfinite(cost):raise ValueError('Nonfinite objective')
            evaluated.append(Individual(frozen,cost))
        evaluated.sort(key=lambda p:p.cost)
        if best is None or evaluated[0].cost<best.cost:best=evaluated[0]
        history.append(dict(generation=generation,best_cost=best.cost,best_genes=best.genes,
                            distinct_population=len({p.genes for p in evaluated})))
        if on_generation:on_generation(history[-1])
        if generation==generations-1:break
        next_population=[np.array(p.genes) for p in evaluated[:2]]
        def parent():
            choices=rng.choice(len(evaluated),3,replace=False)
            return np.array(min((evaluated[i] for i in choices),key=lambda p:p.cost).genes)
        while len(next_population)<population_size-1:
            a,b=parent(),parent();alpha=rng.uniform(-.15,1.15,size=len(low))
            child=alpha*a+(1-alpha)*b
            mask=rng.random(len(low))<.35
            sigma=.12-.08*generation/max(1,generations-1)
            child+=mask*rng.normal(0.,sigma,size=len(low))*span
            next_population.append(np.clip(child,low,high))
        next_population.append(rng.uniform(low,high))
        population=next_population
    return best,history
