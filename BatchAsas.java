// Simcenter STAR-CCM+ macro: BatchAsas.java  (testada no STAR-CCM+ 21.04)
//
// O QUE FAZ
//   Roda todas as geometrias .x_t de GEOM_DIR no .sim base aberto: troca a asa no Subtract,
//   gera a malha, roda o solver com criterio de parada proprio, exporta resumo.csv + imagens
//   de todas as cenas e salva um .sim por asa em OUT_DIR\<nome>\.
//
// REQUISITOS DO .sim BASE (o que estiver aberto quando a macro roda)
//   1. Uma unica operacao Subtract (bloco do tunel menos a asa) e uma regiao criada a partir dela.
//   2. A asa da base e uma peca chamada NOME_ASA com uma superficie unica tambem chamada NOME_ASA,
//      de modo que o contorno da asa na regiao seja "<NOME_ASA>.<NOME_ASA>". A nova asa recebe os
//      mesmos nomes, entao reports, cenas e controles de malha continuam apontando para ela.
//   3. Reports de forca REPORT_LIFT e REPORT_DRAG nesse contorno (a macro grava downforce = -Lift).
//      REPORT_CP (Center of Loads) e opcional.
//   4. Malha, fisica, condicoes de contorno e cenas ja configuradas: a macro so troca a geometria.
//   5. Meio modelo com plano de simetria (forcas x FATOR_SIMETRIA; modelo inteiro: FATOR_SIMETRIA = 1).
//
// REQUISITOS DOS CADs (.x_t)
//   - Exportados ja na posicao final, no mesmo sistema de coordenadas e unidade da asa da base:
//     esta macro NAO translada nem rotaciona a geometria.
//   - Asa inteira ou meia asa: tanto faz, o Subtract corta o que passar do plano de simetria.
//   - Nome do arquivo sem acento (o leitor Parasolid do STAR pode falhar com acentos).
//   Com SO_VERIFICAR = true a macro apenas importa cada geometria, mede a bounding box e grava
//   verificacao.csv, sem malhar nem rodar: use para conferir se os CADs estao no lugar certo.
//
// USO
//   Ajuste a secao "Configuracao" e rode pela GUI (File > Macro > Play Macro, com o .sim base aberto)
//   ou em batch:
//     starccm+ -batch BatchAsas.java -np <nucleos> <Base.sim>
//   (com licenca Power-on-Demand: acrescente -power -podkey <CHAVE>)
package macro;

import java.io.*;
import java.util.*;

import star.common.*;
import star.base.neo.*;
import star.base.report.*;
import star.flow.*;
import star.meshing.*;
import star.vis.*;

public class BatchAsas extends StarMacro {

  // ---- Configuracao -------------------------------------------------------
  // Pasta com os .x_t e pasta de saida (podem ser a mesma).
  static final String GEOM_DIR = "C:\\caminho\\para\\geometrias";
  static final String OUT_DIR = "C:\\caminho\\para\\resultados";
  static final boolean SO_VERIFICAR = false;
  // Se preenchido, roda so a geometria com esse nome (sem .x_t). Vazio = todas.
  static final String SOMENTE = "";
  // Geometrias puladas quando SOMENTE esta vazio (nomes sem .x_t).
  static final List<String> EXCLUIR = Arrays.asList();

  static final String NOME_ASA = "Asa Dianteira";
  static final String REPORT_LIFT = "Lift";
  static final String REPORT_DRAG = "Drag";
  static final String REPORT_CP = "Center of Loads 1";

  // Parada: no minimo MIN_ITER, no maximo MAX_ITER. Para antes se, nas ultimas
  // JANELA iteracoes, Lift e Drag variarem menos que TOL (relativo a media).
  static final int MIN_ITER = 300;
  static final int MAX_ITER = 1000;
  static final int JANELA = 100;
  static final int PASSO = 10;          // iteracoes entre amostras
  static final double TOL = 0.005;      // 0,5 %
  static final double FATOR_SIMETRIA = 2.0;

  static final int IMG_W = 1920;
  static final int IMG_H = 1080;
  // -------------------------------------------------------------------------

  Simulation sim;

  @Override
  public void execute() {
    sim = getActiveSimulation();

    File[] geoms = new File(GEOM_DIR).listFiles((d, n) -> n.toLowerCase().endsWith(".x_t")
        && (SOMENTE.isEmpty()
            ? !EXCLUIR.contains(n.replaceAll("(?i)\\.x_t$", ""))
            : n.equalsIgnoreCase(SOMENTE + ".x_t")));
    if (geoms == null || geoms.length == 0) {
      sim.println("Nenhum .x_t encontrado em " + GEOM_DIR);
      return;
    }
    Arrays.sort(geoms);
    new File(OUT_DIR).mkdirs();

    if (SO_VERIFICAR) {
      verificar(geoms);
      return;
    }

    File resumo = new File(OUT_DIR, "resumo.csv");
    if (!resumo.exists()) {
      escreverLinha(resumo, "geometria,status,parada,iteracoes,celulas,"
          + "drag_meio_N,lift_meio_N,drag_N,downforce_N,eficiencia,"
          + "cp_x_m,cp_y_m,cp_z_m,bbox,tempo_malha_s,tempo_solver_s,tempo_total_s,"
          + "drag_ultimo_N,downforce_ultimo_N,osc_drag_pct,osc_lift_pct");
    }
    registrarBaseManual(resumo);

    for (File geom : geoms) {
      String nome = geom.getName().replaceAll("(?i)\\.x_t$", "");
      sim.println("=== " + nome + " ===");
      long t0 = System.currentTimeMillis();
      try {
        escreverLinha(resumo, rodarCaso(geom, nome, t0));
      } catch (Exception e) {
        sim.println("ERRO em " + nome + ": " + e);
        double total = (System.currentTimeMillis() - t0) / 1000.0;
        escreverLinha(resumo, nome + ",ERRO: " + csv(String.valueOf(e.getMessage()))
            + ",,,,,,,,,,,,,,," + fmt(total) + ",,,,");
      }
    }
    sim.println("Batch concluido. Resumo em " + resumo.getPath());
  }

  String rodarCaso(File geom, String nome, long t0) throws Exception {
    File pasta = new File(OUT_DIR, nome);
    pasta.mkdirs();

    // 1) Trocar a geometria da asa no Subtract
    GeometryPart novaAsa = importarAsa(geom);
    String bbox = bboxTexto(novaAsa);
    trocarAsaNoSubtract(novaAsa);

    // 2) Malha
    sim.getSolution().clearSolution(Solution.Clear.History, Solution.Clear.Fields);
    long tMalha = System.currentTimeMillis();
    sim.getMeshPipelineController().generateVolumeMesh();
    double tempoMalha = (System.currentTimeMillis() - tMalha) / 1000.0;
    long celulas = ((FvRepresentation) sim.getRepresentationManager().getObject("Volume Mesh")).getCellCount();

    // 3) Solver
    sim.getSolution().initializeSolution();
    Report lift = sim.getReportManager().getReport(REPORT_LIFT);
    Report drag = sim.getReportManager().getReport(REPORT_DRAG);
    LinkedList<double[]> amostras = new LinkedList<>();
    int amostrasJanela = Math.max(1, JANELA / PASSO);
    SimulationIterator it = sim.getSimulationIterator();
    String parada = "limite";
    long tSolver = System.currentTimeMillis();
    while (it.getCurrentIteration() < MAX_ITER) {
      it.step(Math.min(PASSO, MAX_ITER - it.getCurrentIteration()));
      amostras.add(new double[] {lift.getReportMonitorValue(), drag.getReportMonitorValue()});
      if (amostras.size() > amostrasJanela) {
        amostras.removeFirst();
      }
      if (it.getCurrentIteration() >= MIN_ITER && amostras.size() == amostrasJanela
          && estavel(amostras, 0) && estavel(amostras, 1)) {
        parada = "convergiu";
        break;
      }
    }
    double tempoSolver = (System.currentTimeMillis() - tSolver) / 1000.0;
    int iteracoes = it.getCurrentIteration();

    // 4) Resultados (media da janela final; valores do meio modelo)
    double liftMeio = media(amostras, 0);
    double dragMeio = media(amostras, 1);
    double dragTotal = dragMeio * FATOR_SIMETRIA;
    double downforce = -liftMeio * FATOR_SIMETRIA;
    double eficiencia = dragTotal != 0 ? downforce / dragTotal : Double.NaN;
    Vector3 cp = ((CenterOfLoadsReport) sim.getReportManager().getReport(REPORT_CP)).getNetLocation();

    // 5) Imagens e .sim
    for (SceneBase sb : sim.getSceneManager().getObjects()) {
      if (sb instanceof Scene) {
        String arq = nome + "_" + sb.getPresentationName().replaceAll("[^A-Za-z0-9._-]", "_") + ".png";
        ((Scene) sb).printAndWait(new File(pasta, arq).getPath(), 1, IMG_W, IMG_H, true, false);
      }
    }
    sim.saveState(new File(pasta, nome + ".sim").getPath());

    // Ultima iteracao e oscilacao na janela final (mesmas colunas da linha Base_manual)
    double[] ultimo = amostras.getLast();
    double total = (System.currentTimeMillis() - t0) / 1000.0;
    return String.join(",", nome, "ok", parada, String.valueOf(iteracoes), String.valueOf(celulas),
        fmt(dragMeio), fmt(liftMeio), fmt(dragTotal), fmt(downforce), fmt(eficiencia),
        fmt(cp.x), fmt(cp.y), fmt(cp.z), csv(bbox), fmt(tempoMalha), fmt(tempoSolver), fmt(total),
        fmt(ultimo[1] * FATOR_SIMETRIA), fmt(-ultimo[0] * FATOR_SIMETRIA),
        fmt(oscilacao(amostras, 1) * 100), fmt(oscilacao(amostras, 0) * 100));
  }

  // Grava os valores da simulacao manual que ja vem resolvida no .sim base, para comparar
  // com a mesma asa rodada pela macro. Usa o mesmo criterio da macro: media das ultimas
  // JANELA (100) iteracoes, amostradas a cada PASSO (10), lidas do historico dos monitores.
  void registrarBaseManual(File resumo) {
    try {
      int iter = sim.getSimulationIterator().getCurrentIteration();
      if (iter == 0) {
        return;
      }
      // Ja registrada numa execucao anterior: nao duplica
      for (String l : java.nio.file.Files.readAllLines(resumo.toPath(), java.nio.charset.StandardCharsets.UTF_8)) {
        if (l.startsWith("Base_manual,")) {
          return;
        }
      }
      List<double[]> amostras = amostrasDoHistorico();
      double liftMeio = media(amostras, 0);
      double dragMeio = media(amostras, 1);
      double[] ultimo = amostras.get(0);
      double dragTotal = dragMeio * FATOR_SIMETRIA;
      double downforce = -liftMeio * FATOR_SIMETRIA;
      Vector3 cp = ((CenterOfLoadsReport) sim.getReportManager().getReport(REPORT_CP)).getNetLocation();
      long celulas = ((FvRepresentation) sim.getRepresentationManager().getObject("Volume Mesh")).getCellCount();
      escreverLinha(resumo, String.join(",", "Base_manual", "ok", "manual", String.valueOf(iter), String.valueOf(celulas),
          fmt(dragMeio), fmt(liftMeio), fmt(dragTotal), fmt(downforce), fmt(dragTotal != 0 ? downforce / dragTotal : Double.NaN),
          fmt(cp.x), fmt(cp.y), fmt(cp.z), "", "", "", "",
          fmt(ultimo[1] * FATOR_SIMETRIA), fmt(-ultimo[0] * FATOR_SIMETRIA),
          fmt(oscilacao(amostras, 1) * 100), fmt(oscilacao(amostras, 0) * 100)));
    } catch (Exception e) {
      sim.println("Nao foi possivel registrar a simulacao manual: " + e);
    }
  }

  // Amostras {lift, drag} a cada PASSO iteracoes nas ultimas JANELA do historico dos
  // monitores, terminando na ultima iteracao (mesma amostragem do solver da macro e da
  // AtualizarCSV). A primeira amostra da lista e a ultima iteracao.
  List<double[]> amostrasDoHistorico() {
    double[] lift = historicoMonitor(REPORT_LIFT);
    double[] drag = historicoMonitor(REPORT_DRAG);
    int len = Math.min(lift.length, drag.length);
    int n = Math.min(JANELA / PASSO, (len + PASSO - 1) / PASSO);
    List<double[]> amostras = new ArrayList<>();
    for (int k = 0; k < n; k++) {
      amostras.add(new double[] {lift[lift.length - 1 - k * PASSO], drag[drag.length - 1 - k * PASSO]});
    }
    return amostras;
  }

  double[] historicoMonitor(String report) {
    for (Monitor m : sim.getMonitorManager().getMonitors()) {
      if (m instanceof ReportMonitor
          && ((ReportMonitor) m).getReport().getPresentationName().equals(report)) {
        double[] y = ((ReportMonitor) m).getAllYValues();
        if (y.length > 0) {
          return y;
        }
      }
    }
    throw new RuntimeException("monitor do report '" + report + "' nao encontrado ou vazio");
  }

  // Importa o .x_t, junta todos os corpos numa peca e todas as superficies numa so.
  GeometryPart importarAsa(File geom) {
    SimulationPartManager partes = sim.get(SimulationPartManager.class);
    Set<String> antes = new HashSet<>();
    for (GeometryPart p : partes.getParts()) {
      antes.add(p.getPresentationName());
    }

    // Mesmas opcoes de uma importacao de CAD gravada pela GUI
    sim.get(PartImportManager.class).importCadPart2(resolvePath(geom.getPath()), "SharpEdges", 30.0, 2, true, 1.0E-5, true, false, false, false, true, true, false, false, false, CadImportUtilities.SimplifyAssemblyNameRetentionOption.IMMEDIATE_ASSEMBLY_NAME);

    List<GeometryPart> novas = new ArrayList<>();
    for (GeometryPart p : partes.getParts()) {
      if (!antes.contains(p.getPresentationName())) {
        novas.add(p);
      }
    }
    if (novas.isEmpty()) {
      throw new RuntimeException("importacao nao criou nenhuma peca");
    }
    GeometryPart asa = novas.get(0);
    if (novas.size() > 1) {
      sim.get(MeshPartFactory.class).combineMeshParts(asa, novas.subList(1, novas.size()));
    }

    MeshPart mp = (MeshPart) asa;
    List<PartSurface> sups = new ArrayList<>(mp.getPartSurfaceManager().getPartSurfaces());
    if (sups.size() > 1) {
      mp.combinePartSurfaces(sups);
    }
    for (PartSurface s : mp.getPartSurfaceManager().getPartSurfaces()) {
      s.setPresentationName(NOME_ASA);
    }
    return asa;
  }

  // Coloca a nova asa no lugar da antiga nas entradas do Subtract e apaga a antiga.
  void trocarAsaNoSubtract(GeometryPart novaAsa) {
    SubtractPartsOperation sub = null;
    for (MeshOperation op : sim.get(MeshOperationManager.class).getObjects()) {
      if (op instanceof SubtractPartsOperation) {
        sub = (SubtractPartsOperation) op;
      }
    }
    if (sub == null) {
      throw new RuntimeException("operacao Subtract nao encontrada no .sim");
    }

    MeshPart bloco = sub.getTargetPart();
    List<GeometryPart> antigas = new ArrayList<>();
    for (GeometryObject go : sub.getInputGeometryObjects().getObjects()) {
      if (go instanceof GeometryPart && !go.getPresentationName().equals(bloco.getPresentationName())) {
        antigas.add((GeometryPart) go);
      }
    }
    sub.getInputGeometryObjects().addPart(novaAsa);
    sub.getInputGeometryObjects().removeParts(antigas);

    // Displayers de cena que mostravam a asa antiga (ex.: Geometry Scene) ficariam
    // vazios ao apagar a peca; guarda quais eram para colocar a nova asa neles.
    Set<String> nomesAntigas = new HashSet<>();
    for (GeometryPart p : antigas) {
      nomesAntigas.add(p.getPresentationName());
    }
    List<PartDisplayer> dispPeca = new ArrayList<>();
    List<PartDisplayer> dispSup = new ArrayList<>();
    for (Scene cena : sim.getSceneManager().getScenes()) {
      for (DisplayerBase d : cena.getDisplayerManager().getObjects()) {
        if (!(d instanceof PartDisplayer)) {
          continue;
        }
        for (Object o : ((PartDisplayer) d).getInputParts().getObjects()) {
          if (o instanceof GeometryPart && nomesAntigas.contains(((GeometryPart) o).getPresentationName())) {
            dispPeca.add((PartDisplayer) d);
            break;
          }
          if (o instanceof PartSurface && nomesAntigas.contains(((PartSurface) o).getPart().getPresentationName())) {
            dispSup.add((PartDisplayer) d);
            break;
          }
        }
      }
    }

    if (!antigas.isEmpty()) {
      sim.deleteObjects(new ArrayList<ClientServerObject>(antigas));
    }
    for (PartDisplayer d : dispPeca) {
      List<NamedObject> objs = new ArrayList<>(d.getInputParts().getObjects());
      objs.add(novaAsa);
      d.getInputParts().setObjects(objs);
    }
    for (PartDisplayer d : dispSup) {
      List<NamedObject> objs = new ArrayList<>(d.getInputParts().getObjects());
      objs.addAll(((MeshPart) novaAsa).getPartSurfaceManager().getPartSurfaces());
      d.getInputParts().setObjects(objs);
    }
    // O nome da peca define o nome do contorno ("Asa Dianteira.Asa Dianteira"),
    // usado pelos relatorios e cenas.
    ((NamedObject) novaAsa).setPresentationName(NOME_ASA);
  }

  void verificar(File[] geoms) {
    File saida = new File(OUT_DIR, "verificacao.csv");
    escreverLinha(saida, "geometria,pecas_importadas,bbox");
    for (File geom : geoms) {
      String nome = geom.getName().replaceAll("(?i)\\.x_t$", "");
      try {
        GeometryPart asa = importarAsa(geom);
        escreverLinha(saida, nome + ",ok," + csv(bboxTexto(asa)));
        sim.deleteObjects(new ArrayList<ClientServerObject>(Arrays.asList(asa)));
      } catch (Exception e) {
        escreverLinha(saida, nome + ",ERRO: " + csv(String.valueOf(e.getMessage())) + ",");
      }
    }
    sim.println("Verificacao concluida: " + saida.getPath());
  }

  // Bounding box da descricao da peca. A ordem dos 6 valores nao e documentada
  // na API, por isso o vetor e gravado inteiro e interpretado na verificacao.
  String bboxTexto(GeometryPart p) {
    try {
      RootDescriptionSource raiz = sim.get(SimulationMeshPartDescriptionSourceManager.class).getRootDescriptionSource();
      DoubleVector bb = ((MeshPart) p).getDescriptions().getCurrentDescription(raiz).getBoundingBox();
      StringBuilder sb = new StringBuilder();
      for (int i = 0; i < bb.size(); i++) {
        sb.append(i > 0 ? " " : "").append(fmt(bb.get(i)));
      }
      return sb.toString();
    } catch (Exception e) {
      return "indisponivel";
    }
  }

  static boolean estavel(List<double[]> amostras, int k) {
    return media(amostras, k) != 0 && oscilacao(amostras, k) < TOL;
  }

  // (max - min) / |media| na janela
  static double oscilacao(List<double[]> amostras, int k) {
    double min = Double.MAX_VALUE, max = -Double.MAX_VALUE;
    for (double[] a : amostras) {
      min = Math.min(min, a[k]);
      max = Math.max(max, a[k]);
    }
    return (max - min) / Math.abs(media(amostras, k));
  }

  static double media(List<double[]> amostras, int k) {
    double s = 0;
    for (double[] a : amostras) {
      s += a[k];
    }
    return amostras.isEmpty() ? Double.NaN : s / amostras.size();
  }

  static String fmt(double v) {
    return String.format(Locale.US, "%.6g", v);
  }

  static String csv(String s) {
    return "\"" + s.replace("\"", "'") + "\"";
  }

  static void escreverLinha(File f, String linha) {
    try (PrintWriter w = new PrintWriter(new OutputStreamWriter(new FileOutputStream(f, true), "UTF-8"))) {
      w.println(linha);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }
}
