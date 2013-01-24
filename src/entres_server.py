#!/usr/bin/env python
import roslib; roslib.load_manifest('entres')

from entres.srv import Infer, InferResponse
from entres.msg import InferenceResult
import rospy

import sys
import os
import time
import traceback
#import configMLN as config
import MLN
import subprocess
import shlex

import tempfile

def readAlchemyResults(output):
    f = file(output, "r")
    results = []
    while True:
        l = f.readline().strip().split(" ")
        if len(l) != 2: break
        atom = l[0]
        prob = float(l[1])
        results.append((atom, prob))
    f.close()
    return results

# --- inference class ---

class MLNInfer(object):
    def __init__(self):
        self.pymlns_methods = MLN.InferenceMethods.getNames()
        self.alchemy_methods = {"MC-SAT":"-ms", "Gibbs sampling":"-p", "simulated tempering":"-simtp", "MaxWalkSAT (MPE)":"-a", "belief propagation":"-bp"}
        self.jmlns_methods = {"MaxWalkSAT (MPE)":"-mws", "MC-SAT":"-mcsat", "Toulbar2 B&B (MPE)":"-t2"}
        self.default_settings = {"numChains":"1", "maxSteps":"", "saveResults":False, "convertAlchemy":False, "openWorld":True} # a minimal set of settings required to run inference
    
    def run(self, mlnFiles, evidenceDB, method, queries, engine="PyMLNs", output_filename=None, params="", **settings):
        '''
            runs an MLN inference method with the given parameters
        
            mlnFiles: list of one or more MLN input files
            evidenceDB: name of the MLN database file from which to read evidence data
            engine: either "PyMLNs"/"internal", "J-MLNs" or one of the keys in the configured Alchemy versions (see configMLN.py)
            method: name of the inference method
            queries: comma-separated list of queries
            output_filename (compulsory only when using Alchemy): name of the file to which to save results
                For the internal engine, specify saveResults=True as an additional settings to save the results
            params: additional parameters to pass to inference method. For the internal engine, it is a comma-separated
                list of assignments to parameters (dictionary-type string), for the others it's just a string of command-line
                options to pass on
            settings: additional settings that control the inference process, which are usually set by the GUI (see code)
                
            returns a mapping (dictionary) from ground atoms to probability values.
                For J-MLNs, results are only returned if settings are saved to a file (settings["saveResults"]=True and output_filename given)
        '''
        self.settings = dict(self.default_settings)        
        self.settings.update(settings)
        input_files = mlnFiles
        db = evidenceDB
        query = queries
        
        results_suffix = ".results"
        output_base_filename = output_filename
        if output_base_filename[-len(results_suffix):] == results_suffix:
            output_base_filename = output_base_filename[:-len(results_suffix)]
        
        # determine closed-world preds
        cwPreds = []
        if "cwPreds" in self.settings:            
            cwPreds = filter(lambda x: x != "", map(str.strip, self.settings["cwPreds"].split(",")))
        haveOutFile = False
        results = None
        
        # engine-specific handling
        if engine in ("internal", "PyMLNs"): 
            try:
                print "\nStarting %s...\n" % method
                
                # read queries
                queries = []
                q = ""
                for s in map(str.strip, query.split(",")):
                    if q != "": q += ','
                    q += s
                    if MLN.balancedParentheses(q):
                        queries.append(q)
                        q = ""
                if q != "": raise Exception("Unbalanced parentheses in queries!")
                
                # create MLN
                verbose = True
                print input_files
                mln = MLN.MLN(input_files, verbose=verbose, defaultInferenceMethod=MLN.InferenceMethods.byName(method))
                
                # set closed-world predicates
                for pred in cwPreds:
                    mln.setClosedWorldPred(pred)
                
                # create ground MRF
                mrf = mln.groundMRF(db, verbose=verbose)
                
                # collect inference arguments
                args = {"details":True, "verbose":verbose, "shortOutput":True, "debugLevel":1}
                args.update(eval("dict(%s)" % params)) # add additional parameters
                if args.get("debug", False) and args["debugLevel"] > 1:
                    print "\nground formulas:"
                    mrf.printGroundFormulas()
                    print
                if self.settings["numChains"] != "":
                    args["numChains"] = int(self.settings["numChains"])
                if self.settings["maxSteps"] != "":
                    args["maxSteps"] = int(self.settings["maxSteps"])
                outFile = None
                if self.settings["saveResults"]:
                    haveOutFile = True
                    outFile = file(output_filename, "w")
                    args["outFile"] = outFile
                args["probabilityFittingResultFileName"] = output_base_filename + "_fitted.mln"

                # check for print/write requests
                if "printGroundAtoms" in args:
                    if args["printGroundAtoms"]:
                        mrf.printGroundAtoms()
                if "printGroundFormulas" in args:
                    if args["printGroundFormulas"]:
                        mrf.printGroundFormulas()
                if "writeGraphML" in args:
                    if args["writeGraphML"]:
                        graphml_filename = output_base_filename + ".graphml"
                        print "writing ground MRF as GraphML to %s..." % graphml_filename
                        mrf.writeGraphML(graphml_filename)
                    
                # invoke inference and retrieve results
                mrf.infer(queries, **args)
                results = {}
                for gndFormula, p in mrf.getResultsDict().iteritems():
                    results[str(gndFormula)] = p
                
                # close output file and open if requested
                if outFile != None:
                    outFile.close()
            except:
                cls, e, tb = sys.exc_info()
                sys.stderr.write("Error: %s\n" % str(e))
                traceback.print_tb(tb)
                
        elif engine == "J-MLNs": # engine is J-MLNs (ProbCog's Java implementation)
            
            # create command to execute
            app = "MLNinfer"
            params = [app, "-i", ",".join(input_files), "-e", db, "-q", query, self.jmlns_methods[method]] + shlex.split(params)
            if self.settings["saveResults"]:
                params += ["-r", output_filename]
            if self.settings["maxSteps"] != "":
                params += ["-maxSteps", self.settings["maxSteps"]]
            if len(cwPreds) > 0:
                params += ["-cw", ",".join(cwPreds)]
            outFile = None
            if self.settings["saveResults"]:
                outFile = output_filename
                params += ["-r", outFile]
            
            # execute
            params = map(str, params)
            print "\nStarting J-MLNs..."
            print "\ncommand:\n%s\n" % " ".join(params)
            t_start = time.time()
            call(params)
            t_taken = time.time() - t_start
            
            if outFile is not None:
                results = dict(readAlchemyResults(outFile))
            
        return results

class EntResServer:
  def __init__(self):
    self.inference = MLNInfer()

  def create_db(self,req):
    # add domain decls for old and new clusters
    oCs = "oldClusters={O" + \
          ",O".join([str(i) for i in range(req.num_old_clusters)]) + \
          "}\n"
    nCs = "newClusters={N" + \
          ",N".join([str(i) for i in range(req.num_new_clusters)]) + \
          "}\n"
    self.db = [oCs, nCs, "\n"]

    # add similarities in appearance and position
    for o in range(req.num_old_clusters):
      oC = "O%i" % o
      for n in range(req.num_new_clusters):
        nC = "N%i" % n
        idx = o * req.num_old_clusters + n
        simA = req.similarAppearance[idx]
        simP = req.similarPose[idx]
        self.db.append ("%f looksSimilar(%s,%s)\n" % (simA,oC,nC))
        self.db.append ("%f samePos(%s,%s)\n" % (simP,oC,nC))

    # add out of view clusters
    self.db.append("\n")
    for unobserved in req.out_of_view:
      self.db.append("outOfView(O%i)" % unobserved)

  def handle_inference(self,req):
    print req.model
    print req.num_new_clusters
    print req.num_old_clusters
    print req.similarAppearance
    print req.similarPose
    print req.unobserved
    print req.queries

    self.create_db(req)
    input_files = [req.model]

    db = ""
    with tempfile.NamedTemporaryFile(mode='w', prefix='tmp_er_', delete=False) as f:
      for line in self.db:
        f.write(line)
      db = f.name

    method = "MC-SAT"
    params = ""
    

    try:
      self.inference.run(input_files, db, method, self.settings["query"], params=params, **self.settings)
    except:
      cls, e, tb = sys.exc_info()
      sys.stderr.write("Error: %s\n" % str(e))
      traceback.print_tb(tb)


    
    resp = InferResponse()
    r = InferenceResult()
    r.functionName = "is"
    r.params=["o1","n3"]
    r.probability = 0.99
    resp.results = [r]
    resp.persistent = [1,2]
    resp.new_to_old = [3,4]

    return resp

  def serve(self):
    rospy.init_node('entres_server')
    rospy.Service('infer', Infer, self.handle_inference)
    print "Ready to resolve some identities!"
    rospy.spin()

if __name__ == "__main__":
  ERS = EntResServer()
  ERS.serve()

