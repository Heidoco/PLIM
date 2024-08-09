#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <ilcplex/ilocplex.h>

using namespace std;
ILOSTLBEGIN

int main(int argc, char *argv[])
{
	try
	{
		//Leitura dos dados
		ifstream arq(argv[1]);
		if (!arq.is_open())
		{
			cout << "Error openning file: " << argv[1] << endl;
			arq.close();
			exit(EXIT_FAILURE);
		}

		int n; 											// Quantidade de nós
		arq >> n;
		float alpha;									// Fator de desconto no custo de transporte em um link entre hubs
		vector<double> codx(n);							// Coordenada x dos nós (AP)
		vector<double> cody(n);							// Coordenada y dos nós (AP)
		vector<vector<double>> w(n, vector<double>(n)); // declara um vetor de vetores para representar uma matriz com n linhas e n colunas - Quantidade de demanda a ser enviada entre os nós i e j
		vector<vector<double>> r(n, vector<double>(n)); // Receita obtida por enviar uma unidade de demanda entre os nós i e j
		vector<vector<double>> c(n, vector<double>(n)); // Custos por enviar uma unidade de demanda entre os nós i e j
		vector<vector<double>> q(n, vector<double>(n)); // Custos operação de uma conexão direta entre os nós i e j
		vector<double> s(n);							// Custos fixos de instalação de um hub
		vector<vector<double>> g(n, vector<double>(n)); // Custos de operação nos links inter-hubs
		float soma = 0;									// Soma para obter a média geral dos custos fixos de instalação hub - Dados AP

		// COLETAR DADOS CAB (Rodar: ./form2 cabn.txt 0.8 1000 150 15 3)

		// Coletar demanda e custos CAB
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				arq >> w[i][j];
				arq >> c[i][j];
				soma = soma + w[i][j];
			}
		}

		// Escalonando demanda CAB
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				w[i][j] = w[i][j] / soma;
			}
		}

		// Coletar fator de desconto nos links inter-hubs CAB
		if (argc >= 3)
			alpha = atof(argv[2]);
		else
			alpha = 0.2;

		// Coletar receita CAB
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				r[i][j] = atoi(argv[3]);
			}
		}

		// Coletar custo fixo de instalação CAB
		for (int i = 0; i < n; i++)
		{
			s[i] = atoi(argv[4]);
		}

		// Coletar custo de operar links inter-hubs CAB
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				g[i][j] = atoi(argv[5]);
			}
		}

		// Coletar custo de operar links diretos CAB
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				q[i][j] = atoi(argv[6]);
			}
		}

		// Declaração do modelo

		IloEnv env;
		IloModel mod(env);
		IloCplex cplex(mod);

		IloNumVarArray h(env, n, 0, 1, ILOBOOL); // h[k] indica se um hub é localizado no nó k

		IloArray<IloNumVarArray> z(env, n); // z[k][l] indica se um link inter-hub é operado entre os hubs l e k
		for (int k = 0; k < n; k++)
			z[k] = IloNumVarArray(env, n, 0, 1, ILOBOOL);

		IloArray<IloArray<IloArray<IloNumVarArray>>> x(env, n); // x[i][j][k][l] representa a fração da demanda entre os nós i e j que é roteada entre os hubs k e l
		for (int i = 0; i < n; i++)
		{
			x[i] = IloArray<IloArray<IloNumVarArray>>(env, n);
			for (int j = 0; j < n; j++)
			{
				x[i][j] = IloArray<IloNumVarArray>(env, n);
				for (int k = 0; k < n; k++)
				{
					x[i][j][k] = IloNumVarArray(env, n, 0, IloInfinity, ILOFLOAT);
				}
			}
		}

		IloArray<IloArray<IloNumVarArray>> a(env, n); // a[i][j][k] é a fração da demanda entre os nós i e j que é roteada através de um caminho no qual o primeiro hub é k
		for (int i = 0; i < n; i++)
		{
			a[i] = IloArray<IloNumVarArray>(env, n);
			for (int j = 0; j < n; j++)
			{
				a[i][j] = IloNumVarArray(env, n, 0, IloInfinity, ILOFLOAT);
			}
		}

		IloArray<IloArray<IloNumVarArray>> b(env, n); // b[i][j][l] é a fração da demanda entre os nós i e j que é roteada através de um caminho no qual o último hub é l
		for (int i = 0; i < n; i++)
		{
			b[i] = IloArray<IloNumVarArray>(env, n);
			for (int j = 0; j < n; j++)
			{
				b[i][j] = IloNumVarArray(env, n, 0, IloInfinity, ILOFLOAT);
			}
		}

		IloArray<IloNumVarArray> e(env, n); // e[i][j] indica a fração da demanda que é roteada entre os nós não concentradores - conexão direta
		for (int i = 0; i < n; i++)
			e[i] = IloNumVarArray(env, n, 0, 1, ILOBOOL);

		// ====================================Formulação - Conexão Direta=================================================
		// maximize sum(i in 1..n, j in 1..n, k in 1..n, l in 1..n) r[i][j] * w[i][j] * a[i][j][k] + r[i][j]*w[i][j]*e[i][j]
		// - [ sum(i in 1..n, j in 1..n, k in 1..n) c[i][k] * w[i][j] * a[i][j][k]
		// + sum(i in 1..n, j in 1..n, l in 1..n) c[l][j] * w[i][j] * b[i][j][l]
		// + sum(i in 1..n, j in 1..n) c[i][j] * w[i][j] * e[i][j]
		// + sum(i in 1..n, j in 1..n, k in 1..n, l in 1..n) alpha * c[k][l] * x[i][j][k][l] + sum(k in 1..n) s[k] * h[k]
		// + sum(k in 1..n, l in 1..n) g[k][l] * z[k][l] + sum(i in 1..n, j in 1..n) q[i][j] * e[i][j] ];
		// ===============================================================================================================

		IloExpr expfo(env);
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				expfo += r[i][j] * w[i][j] * e[i][j] - c[i][j] * w[i][j] * e[i][j];
				for (int k = 0; k < n; k++)
				{
					expfo += (r[i][j] - c[i][k]) * w[i][j] * a[i][j][k];
				}
				for (int l = 0; l < n; l++)
				{
					expfo -= c[l][j] * w[i][j] * b[i][j][l];
				}
				for (int k = 0; k < n; k++)
				{
					for (int l = 0; l < n; l++)
					{
						expfo -= alpha * c[k][l] * w[i][j] * x[i][j][k][l];
					}
				}
			}
		}
		for (int k = 0; k < n; k++)
		{
			expfo -= s[k] * h[k];
			for (int l = 0; l < n; l++)
			{
				expfo -= g[k][l] * z[k][l];
			}
		}
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				expfo -= q[i][j] * e[i][j];
			}
		}

		IloAdd(mod, IloMaximize(env, expfo));
		expfo.end();

		//===========================================================================
		// forall(i in 1..n, j in 1..n)  sum(k in 1..n) a[i][j][k] + e[i][j] <= 1
		//===========================================================================
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				IloExpr r1(env);
				r1 += e[i][j];
				for (int k = 0; k < n; k++)
				{
					r1 += a[i][j][k];
				}
				mod.add(r1 <= 1);
				r1.end();
			}
		}

		//===========================================================================
		// forall(i in 1..n, j in 1..n)  sum(l in 1..n) b[i][j][l] + e[i][j] <= 1
		//===========================================================================
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				IloExpr r2(env);
				r2 += e[i][j];
				for (int l = 0; l < n; l++)
				{
					r2 += b[i][j][l];
				}
				mod.add(r2 <= 1);
				r2.end();
			}
		}

		//==========================================================================================================================================
		// forall(i in 1..n, j in 1..n, k in 1..n)  a[i][j][k] + sum(l in 1..n, l!k) a[i][j][k] + x[i][j][l][k] == b[i][j][k] + sum(l in 1..n, l!k) x[i][j][k][l]
		//==========================================================================================================================================
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				for (int k = 0; k < n; k++)
				{
					IloExpr r3(env);
					r3 += a[i][j][k] - b[i][j][k];
					for (int l = 0; l < n; l++)
					{
						if (l != k)
						{
							r3 += x[i][j][l][k] - x[i][j][k][l];
						}
					}
					mod.add(r3 == 0);
					r3.end();
				}
			}
		}

		//=============================================================
		// forall(i in 1..n, j in 1..n, k in 1..n) a[i][j][k] <= h[k], i != k
		//=============================================================
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				for (int k = 0; k < n; k++)
				{
					mod.add(a[i][j][k] <= h[k]);
				}
			}
		}

		//=============================================================
		// forall(i in 1..n, j in 1..n, l in 1..n) b[i][j][l] <= h[l], j != l
		//=============================================================
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				for (int l = 0; l < n; l++)
				{
					mod.add(b[i][j][l] <= h[l]);
				}
			}
		}

		//==================================================================================================
		// forall(i in 1..n, j in 1..n, k in 1..n, l in 1..n, l!k) x[i][j][k][l] <= z[k][l]
		//==================================================================================================
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				for (int k = 0; k < n; k++)
				{
					for (int l = 0; l < n; l++)
					{
						if (l != k)
							mod.add(x[i][j][k][l] <= z[k][l]);
					}
				}
			}
		}

		//===================================================
		// forall(k in 1..n, l in 1..n, l!k)  z[k][l] <= h[k]
		//===================================================
		for (int k = 0; k < n; k++)
		{
			for (int l = 0; l < n; l++)
			{
				if (l != k)
					mod.add(z[k][l] <= h[k]);
			}
		}

		//===================================================
		// forall(k in 1..n, l in 1..n, l!k)  z[k][l] <= h[l]
		//===================================================
		for (int k = 0; k < n; k++)
		{
			for (int l = 0; l < n; l++)
			{
				if (l != k)
					mod.add(z[k][l] <= h[l]);
			}
		}

		//===================================================
		// forall(i in 1..n, j in 1..n, l!k)  e[i][j] + h[i] <= 1
		//===================================================
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				mod.add(e[i][j] + h[i] <= 1);
			}
		}

		//===================================================
		// forall(i in 1..n, j in 1..n, l!k)  e[i][j] + h[j] <= 1
		//===================================================
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				mod.add(e[i][j] + h[j] <= 1);
			}
		}

		// Configurações do cplex

		// cplex.setParam(IloCplex::EpGap, 0.000001); // Tolerancia
		cplex.setParam(IloCplex::TiLim, 86400); // Tempo limite de resolução
		cplex.setWarning(env.getNullStream());	// Eliminar warnings
		cplex.setOut(env.getNullStream());		// Eliminar os logs do solver
		// cplex.setParam(IloCplex::Param::Benders::Strategy, 3); // Ativar Benders do solver

		// Resolvendo o problema
		IloTimer crono(env); // Coletar o tempo
		double lb = 0;
		double ub = 10e-10;
		double gap;

		// Pré-fixando variáveis
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				for (int k = 0; k < n; k++)
				{
					if (r[i][j] < c[i][k])
					{
						a[i][j][k].setBounds(0, 0);
					}
				}
				for (int l = 0; l < n; l++)
				{
					if (r[i][j] < c[l][j])
					{
						b[i][j][l].setBounds(0, 0);
					}
				}
			}
		}

		// for(int i = 0; i< n; i++){   //Desativar conexões diretas
		//   for(int j = 0; j < n; j++){
		//     e[i][j].setBounds(0, 0);
		//   }
		// }

		crono.start();
		cplex.solve();
		crono.stop();

		if (cplex.getStatus() == IloAlgorithm::Optimal)
		{
			lb = cplex.getObjValue();
			ub = cplex.getObjValue();
			gap = 0.0;
		}

		///=====================================
		/// Salvando os resultados - CAB
		///=====================================

		cout << argv[0] << " " << argv[1] << " " << argv[2] << " " << argv[3] << " " << argv[4] << " " << argv[5] << " " << argv[6] << "\n";

		FILE *re;
		re = fopen("ResultadosNCONDIRETA.txt", "aw+");
		fprintf(re, "Receita: %s\n", argv[3]);
		fprintf(re, "Custos: %s %s %s\n", argv[4], argv[5], argv[6]);
		fprintf(re, "Alpha: %s\n", argv[2]);
		fprintf(re, "Valor função objetivo: %f\n", (double)cplex.getObjValue());

		float cont1 = 0;
		float cont2 = 0;
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				if (cplex.getValue(e[i][j]) > 0.001)
				{
					cont2 = cont2 + 1;
				}
				for (int k = 0; k < n; k++)
				{
					if (cplex.getValue(a[i][j][k]) > 0.001)
					{
						cont1 = cont1 + 1;
					}
				}
			}
		};
		fprintf(re, "Demanda total atendida: %f\n", (cont1 + cont2) / 600);
		fprintf(re, "Demanda por link direto: %f\n", cont2 / 600);
		fprintf(re, "Quantidade de pares atendidos: %f\n", cont1 + cont2);
		fprintf(re, "Hubs: ");
		for (int j = 0; j < n; j++)
		{
			if (cplex.getValue(h[j]) >= 0.1)
			{
				fprintf(re, "%d ", j + 1);
			}
		};
		fprintf(re, "\n");
		fprintf(re, "Tempo de CPU: %f\n", (double)crono.getTime());
		fprintf(re, "======================================================================\n");
	}
	catch (IloException &ex)
	{
		cerr << "Error: " << ex << endl;
	}
	return 0;
}
