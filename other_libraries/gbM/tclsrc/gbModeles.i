/*
 * Copyright (c) 2003 LAAS/CNRS -- RIA --
 * Daniel SIDOBRE -- fevrier 2003
 */

typedef struct Gb_dataMGD {
  double d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12;
  double c1, c2, c3, c4, c5, c6;
  double s1, s2, s3, s4, s5, s6;
  double c23, s23;
  int e1, e2, e3;
} Gb_dataMGD;

typedef enum Gb_statusMGI {
  MGI_OK = 0,
  MGI_ERROR,
  MGI_APPROXIMATE,
  MGI_SINGULAR
} Gb_statusMGI;

typedef struct Gb_6rParameters {
  double a2, r4;
  double epsilon;  /* to determine if the position is singular */
  double of1;
  double of2;
  double of3;
  double of4;
  double of5;
  double of6;
} Gb_6rParameters;

typedef struct Gb_q6 {
  double q1;
  double q2;
  double q3;
  double q4;
  double q5;
  double q6;
  /* ?? pince */
} Gb_q6;

/* inutile */ char* Gb_statusMGI_s(Gb_statusMGI u);

double Gb_atan2(double y, double x);

%inline %{
  void Gb_dataMGD_puts(Gb_dataMGD *d) {
    Gb_dataMGD_print(stdout, d);
  }
%}

void Gb_MGD6rTh(Gb_6rParameters* bras, Gb_q6* eq,
				     Gb_dataMGD* d, Gb_th* th);

void Gb_MGD6r_6Th(Gb_6rParameters* bras, Gb_q6* eq, Gb_dataMGD* d,
                Gb_th* th01, Gb_th* th02, Gb_th* th03, 	
		Gb_th* th04, Gb_th* th05, Gb_th* th06);	

Gb_statusMGI Gb_MGI6rTh(Gb_6rParameters* bras, Gb_th* eth,
				int e1, int e2, int e3, Gb_q6* old_q,
				Gb_dataMGD* d, Gb_q6* sq);

Gb_statusMGI Gb_MGI6rTh_O(Gb_6rParameters* bras, Gb_th* eth,
			  Gb_q6* old_q,
			  Gb_dataMGD* d, Gb_q6* sq);

typedef struct Gb_jac {
  Gb_v6 c1;
  Gb_v6 c2;
  Gb_v6 c3;
  Gb_v6 c4;
  Gb_v6 c5;
  Gb_v6 c6;
} Gb_jac;

void Gb_MDD6r(Gb_6rParameters* bras, Gb_dataMGD* d, Gb_th* t06, 
	      Gb_jac* jac);
