1. smaller barrierBase (e.g., 1) is better for optimization


TODO: update each factor's evaluaterror and getmapfunction
The pseudo-code is briefly outlined there:

VectorDynamic Recover(res, mask, map)

Result=Optimize(tasks)
{
    // declaration
    n: variable dimension
    m: error dimension
    mapVec: mapping function vector, int -> boost::function( input: index(int);output: value of StartTimeVector(double) )
    maskForEliminate: vector<bool> bool;

    MappingFunc: boost::function( input: StartTimeVector; output: StartTimeVector; )
    {
        for( int i=0;i<n;i++)
        {
            res(i,0)=mapVec[i](i);
        }
    }
    mapVec.initialize(initialEstimate)
    resTemp;
    Update(initialEstimate)
    
    while (1)
    {
        

        resTemp, graph = UnitOptimization(tasks, initialEstimate,
                                    MappingFunc, sizeOfVariables, variableDimension, hyperPeriod);
        initialEstimate=resTemp

        bool whetherEliminate=false;
        vector<double> initialEstimateVector;
        initialEstimateVector.reserve(n);
        for(i: n)
        {
            for(factor: graph.factors)
            {
                errorBase=factor.evaluateError(resTemp);
                resTemp[i]+=delta;
                errorCurr= graph.evaluate(resTemp);
                resTemp[i]-=delta;
                for(j:m_factor)
                {
                    if(errorCurr[j]-errorBase[j] > threshold)
                    {
                        success, mapCurr = factor.getMappingFunction(i, resTemp);
                        if(success)
                        {
                            mapVec.push_back(mapCurr);
                            whetherEliminate=true;
                        }
                            
                        else
                            // it seems we only need to skip this constraint
                            initialEstimateVector.push_back(resTemp(i,0));
                            ;
                    }
                }
                
            }
            
        }
        if(not whetherEliminate)
        {
            
            break;
        }


        loopNumber++;
        if (loopNumber > N)
        {
            cout << "Loop number error in OptimizeScheduling" << endl;
            throw;
        }
    }

    // Performance check, etc
    return resTemp;
}