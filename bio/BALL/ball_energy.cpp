// authors :
//   Romain Iehl <riehl@laas.fr>
//   Ron Alterovitz

#include "ball_energy.hpp"

#include <BALL/MOLMEC/MMFF94/MMFF94NonBonded.h>
#include <BALL/MOLMEC/MMFF94/MMFF94.h>
#include <BALL/KERNEL/selector.h>
#include <BALL/KERNEL/PTE.h>
#include <BALL/KERNEL/system.h>
#include <BALL/STRUCTURE/fragmentDB.h>
#include <BALL/FORMAT/PDBFile.h>
#include <BALL/FORMAT/MOL2File.h>
#include <BALL/MOLMEC/MINIMIZATION/shiftedLVMM.h>

#include <queue>
#include <map>
#include <algorithm>

#include "string_func.h"
#include "Planner-pkg.h"
#include "robot.hpp"

using namespace BALL;

const char* proteinFilename = "/home/riehl/my_stuff/molecules/BCL/ortho-test/o_a1rrc_prot_babel.pdb";
const char* ligandFilename = "/home/riehl/my_stuff/molecules/BCL/ortho-test/o_a1rrc_lig.mol2";

Molecule* loadMolecule(std::string filename) {

	Molecule* ligand = NULL;

	if( ends_with(to_lower(filename), ".pdb") ) {

		Log.info() << "Loading molecule from PDB file " << filename << "..." << std::endl;

		ligand = new Molecule();

		try{ 
			PDBFile pdbFile(filename);

			// if the file could not be opened, throw an exception
			if (!pdbFile)
				throw std::runtime_error(std::string("Cannot open PDB file ") + filename);

			// read the contents of the file into the protein
			pdbFile >> *ligand;
			pdbFile.close();
		} catch( Exception::ParseError& e ) {
			delete ligand;
			throw std::runtime_error(std::string("Error opening pdb file ") + filename + std::string(": ") + to_string(e.getMessage()));
		} catch( Exception::FileNotFound& e ) {
			delete ligand;
			throw std::runtime_error(std::string("Error opening pdb file ") + filename + std::string(": ") + to_string(e.getMessage()));
		}

	} else if( ends_with(to_lower(filename), ".mol2") ) {

		Log.info() << "Loading molecule from MOL2 file " << filename << "..." << std::endl;

		System ligandSystem;
		try {
			MOL2File mol2File(filename.c_str());
			if( !mol2File )
				throw std::runtime_error(std::string("Cannot open mol2 file ") + filename);
			// Due to a bug in the BALL library, we cannot directly load the molecule from the mol2 file.
			// We instead load a system from the mol2 file and then extract the molecule from the system.
			mol2File.read(ligandSystem);
			mol2File.close();
		} catch( Exception::ParseError& e ) {
			throw std::runtime_error(std::string("Error opening mol2 file ") + filename + std::string(": ") + to_string(e.getMessage()));
		} catch( Exception::FileNotFound& e ) {
			throw std::runtime_error(std::string("Error opening mol2 file ") + filename + std::string(": ") + to_string(e.getMessage()));
		}

		if( ligandSystem.countAtoms() == 0 )
			throw std::runtime_error(std::string("Did not find any atoms in mol2 file ") + filename);

		if( ligandSystem.countMolecules() == 0 )
			throw std::runtime_error(std::string("Did not find any molecules in mol2 file ") + filename);

		if( ligandSystem.countMolecules() > 1 )
			Log.warn() << "Read more than one molecule in mol2 file " + filename + ", ignoring molecules after the first one." << std::endl;

		// Return the first molecule in the system that was just loaded from the file
		ligand = new Molecule(*ligandSystem.getMolecule(0));

	} else {

		throw std::runtime_error(std::string("File type not recognized based on filename extension (only PDB and MOL2 files are supported): ") + filename);
	}

	return ligand;
}

Protein* loadProtein(std::string filename) {

	Protein *protein = NULL;

	if( ends_with(to_lower(filename), ".pdb") ) {

		Log.info() << "Loading protein from PDB file " << filename << "..." << std::endl;

		protein = new Protein();

		try {
			PDBFile pdbFile(filename);

			// if the file could not be opened, throw an exception
			if (!pdbFile)
				throw std::runtime_error(std::string("Cannot open PDB file ") + filename);

			// read the contents of the file into the protein
			pdbFile >> *protein;
			pdbFile.close();
		} catch( Exception::ParseError& e ) {
			delete protein;
			throw std::runtime_error(std::string("Error opening pdb file ") + filename + std::string(": ") + to_string(e.getMessage()));
		} catch( Exception::FileNotFound& e ) {
			delete protein;
			throw std::runtime_error(std::string("Error opening pdb file ") + filename + std::string(": ") + to_string(e.getMessage()));
		}

	} else if( ends_with(to_lower(filename), ".mol2") ) {

		Log.info() << "Loading protein from MOL2 file " << filename << "..." << std::endl;

		protein = new Protein();

		try {
			MOL2File mol2File(filename.c_str());
			if( !mol2File )
				throw std::runtime_error(std::string("Cannot open mol2 file ") + filename);
			// Due to a bug in the BALL library, we cannot directly load the molecule from the mol2 file.
			// We instead load a system from the mol2 file and then extract the molecule from the system.
			mol2File >> *protein;
			mol2File.close();

			if( protein->countAtoms() == 0 )
				throw std::runtime_error(std::string("Did not find any atoms in MOL2 file ") + filename);

		} catch( Exception::ParseError& e ) {
			delete protein;
			throw std::runtime_error(std::string("Error opening mol2 file ") + filename + std::string(": ") + to_string(e.getMessage()));
		} catch( Exception::FileNotFound& e ) {
			delete protein;
			throw std::runtime_error(std::string("Error opening mol2 file ") + filename + std::string(": ") + to_string(e.getMessage()));
		}

	} else {

		throw std::runtime_error(std::string("File type not recognized based on filename extension (only PDB and MOL2 files are supported): ") + filename);
	}

//writeProtein("temp_mol2.mol2", *protein);

	return protein;
}

struct BallEnergyData
{
  BallEnergyData(p3d_env* env) :
    system(NULL), protein(NULL), ligand(NULL), mmff94(),
    robot(env->robot[0]), lastConfiguration()
  {
  }

  ~BallEnergyData()
  {
  }
  
  BALL::System* system;
  BALL::Protein* protein;
  BALL::Molecule* ligand;
  BALL::MMFF94 mmff94;
  std::map<p3d_poly*, BALL::Atom*> fullMap;
  Robot robot;
  std::tr1::shared_ptr<Configuration> lastConfiguration;
};

bool samePosition(BALL::Atom* a, p3d_poly* p)
{
  return((fabs(p->poly->pos[0][3] - a->getPosition()[0]) +
	  fabs(p->poly->pos[1][3] - a->getPosition()[1]) +
	  fabs(p->poly->pos[2][3] - a->getPosition()[2])) < 1e-2);
}

BallEnergy::BallEnergy(p3d_env* env) :
  d(new BallEnergyData(env))
{
  env->energyComputer = this;
  d->system = new BALL::System();
  cout << "Loading protein file: " << proteinFilename << endl;
  d->protein = loadProtein(proteinFilename);
  //  Selector waterSelector("residue(HOH)");
  //  protein->apply(waterSelector);
  //  protein->removeSelected();
  FragmentDB db("");
  d->protein->apply(db.normalize_names);
  d->protein->apply(db.build_bonds);
  d->system->insert(*d->protein);
  cout << "Loading ligand file: " << ligandFilename << endl;
  d->ligand = loadMolecule(ligandFilename);
  cout << "Num atoms in ligand: " << d->ligand->countAtoms() << endl;
  cout << "Num bonds in ligand: " << d->ligand->countBonds() << endl;
  d->system->insert(*d->ligand);
  //  d->mmff94.options[MMFF94::Option::NONBONDED_CUTOFF] = 500.f;
  //  d->mmff94.removeComponent("MMFF94 StretchBend");
  d->mmff94.setup(*d->system);
  //  d->ligand->select();
  //  d->mmff94.updateEnergy();
  //  d->mmff94.updateForces();
  //  cout << "Initial energy:   " << d->mmff94.getEnergy() << std::endl;
  //  cout << "Number of atoms:   " << d->mmff94.getNumberOfAtoms() << std::endl;
  //  cout << "Number of movable atoms:   " << d->mmff94.getNumberOfMovableAtoms() << std::endl;
  //  cout << "Use selection: " << d->mmff94.getUseSelection() << std::endl;  

  p3d_rob* robotPt = env->robot[0];
  for(int i(0); i < robotPt->no; i++)
  {
    for(int j(0); j < robotPt->o[i]->np; j++)
    {
      bool found(false);
      for(AtomIterator moving_atom_it = d->system->beginAtom();
	  moving_atom_it != d->system->endAtom();
	  moving_atom_it++)
      {
	if(samePosition(&*moving_atom_it, robotPt->o[i]->pol[j]))
	{
	  found = true;
	  d->fullMap[robotPt->o[i]->pol[j]] = &(*moving_atom_it);
	  break;
	}
      }
      if(!found)
      {
	std::cout << "atom not found" << std::endl;
      }
    }
  }
}

void BallEnergy::update(Configuration& conf)
{
  d->robot.setAndUpdate(conf);
  std::vector<p3d_jnt*> changes;
  std::set<p3d_jnt*> propagatedChanges;
  std::vector<p3d_poly*> propagatedPolys;
  if(d->lastConfiguration.get())
  {
    for (int i(0); i <= d->robot.getRobotStruct()->njoints; i++)
    {
      p3d_jnt* joint(d->robot.getRobotStruct()->joints[i]);
      for (int j(0); j < joint->dof_equiv_nbr; j++)
      {
	int k = joint->index_dof + j;
	// Check which dofs have moved
	if(fabs(conf.getConfigStruct()[k] - d->lastConfiguration->getConfigStruct()[k]) > 1e-6)
	{
	  changes.push_back(d->robot.getRobotStruct()->joints[i]);
	  break;
	}
      }
    }

    for(unsigned i(0); i < changes.size(); i++)
    {
      std::queue<p3d_jnt*> remainingJoints;
      if(std::find(propagatedChanges.begin(),
		   propagatedChanges.end(),
		   changes[i]) ==
	 propagatedChanges.end())
      {
	remainingJoints.push(changes[i]);
      }
      while(remainingJoints.size() > 0)
      {
	p3d_jnt* joint = remainingJoints.front();
	remainingJoints.pop();
	if(std::find(propagatedChanges.begin(),
		     propagatedChanges.end(),
		     joint) == propagatedChanges.end())
	{
	  propagatedChanges.insert(joint);
	  for(int j(0); j < joint->o->np; j++)
	  {
	    propagatedPolys.push_back(joint->o->pol[j]);
	  }
	  for(int j(0); j < joint->n_next_jnt; j++)
	  {
	    remainingJoints.push(joint->next_jnt[j]);
	  }
	}
      }
    }
    for(unsigned i(0); i < propagatedPolys.size(); i++)
    {
      d->fullMap[propagatedPolys[i]]->setPosition(Vector3(propagatedPolys[i]->poly->pos[0][3], propagatedPolys[i]->poly->pos[1][3], propagatedPolys[i]->poly->pos[2][3]));
    }
  }
  else 
  {
    for(int i(0); i < d->robot.getRobotStruct()->no; i++)
    {
      for(int j(0); j < d->robot.getRobotStruct()->o[i]->np; j++)
      {
	p3d_poly* p = d->robot.getRobotStruct()->o[i]->pol[j];
	d->fullMap[p]->setPosition(Vector3(p->poly->pos[0][3], p->poly->pos[1][3], p->poly->pos[2][3]));
      }
    }
  }
  d->lastConfiguration = conf.copy();
}
  
double BallEnergy::computeEnergy(Configuration& conf)
{
  this->update(conf);
  return(this->computeEnergy());
}

double BallEnergy::computeEnergy()
{
  d->ligand->select();
  d->mmff94.updateEnergy();
  d->mmff94.updateForces();
  return(d->mmff94.getEnergy());
}
