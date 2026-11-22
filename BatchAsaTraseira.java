// Simcenter STAR-CCM+ macro: BatchAsaTraseira.java  (testada no STAR-CCM+ 21.04)
//
// O QUE FAZ
//   Para cada arquivo Parasolid (.x_t) da pasta GEOM_DIR: troca a asa no Subtract do .sim
//   aberto, gera a malha, roda o solver com criterio de parada proprio, exporta
//   resumo.csv + imagens de todas as cenas e salva um .sim por asa em OUT_DIR\<nome>\.
//   A nova asa entra com uma superficie unica ("Asa" -> contorno "Asa.Asa"); os reports
//   de forca e os displayers que usavam os contornos da asa antiga passam a usar esse contorno.
//
// REQUISITOS DO .sim BASE (o que estiver aberto quando a macro roda)
//   1. Uma unica operacao Subtract (bloco do tunel menos a asa) e uma regiao criada a partir dela.
//   2. A asa da base e uma peca cujas superficies, no Subtract, se chamam "Asa.*"
//      (ou seja, a peca se chama NOME_ASA). Os contornos da asa na regiao tambem "Asa.*".
//   3. Reports de forca chamados REPORT_LIFT e REPORT_DRAG (Lift positivo = sustentacao;
//      a macro grava downforce = -Lift). REPORT_CP (Center of Loads) e opcional.
//   4. Malha, fisica, condicoes de contorno e cenas ja configuradas: a macro so troca a geometria.
//   5. Meio modelo com plano de simetria normal a z (forcas multiplicadas por FATOR_SIMETRIA).
//      Para modelo inteiro use FATOR_SIMETRIA = 1.
//
// REQUISITOS DOS CADs (.x_t)
//   - Mesma unidade e mesmos eixos da asa da base: escoamento em +x, y para cima, envergadura em z.
//     A macro so TRANSLADA a asa (POSICIONAR); ela nao rotaciona. CAD exportado com outro eixo
//     para cima precisa ser corrigido antes.
//   - Asa inteira ou meia asa: tanto faz, o Subtract corta o que passar do plano de simetria.
//   - Nome do arquivo sem acento (o leitor Parasolid do STAR pode falhar com acentos).
//
// POSICIONAMENTO (POSICIONAR = true)
//   Cada asa e transladada para: bordo de ataque (xmin) em REF_XMIN, topo (ymax) em REF_YMAX e
//   centro da envergadura em Z_SIMETRIA. Use os valores da bbox da asa de referencia
//   (Geometry > Parts > peca > botao direito > Bounding Box). Com POSICIONAR = false o CAD entra
//   exatamente onde foi exportado. Rode antes com SO_VERIFICAR = true para conferir as
//   bounding boxes em verificacao.csv sem malhar nem rodar.
//
// USO
//   Ajuste a secao "Configuracao" e rode pela GUI (File > Macro > Play Macro, com o .sim base aberto)
//   ou em batch:
//     starccm+ -batch BatchAsaTraseira.java -np <nucleos> <BaseTraseira.sim>
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

public class BatchAsaTraseira extends StarMacro {

  // ---- Configuracao -------------------------------------------------------
  // Pasta com os .x_t e pasta de saida (podem ser a mesma).
  static final String GEOM_DIR = "C:\\caminho\\para\\geometrias";
  static final String OUT_DIR = "C:\\caminho\\para\\resultados";
  // Se preenchida, roda so essas geometrias (nomes sem .x_t). Vazia = todas menos EXCLUIR.
  static final List<String> RODAR = Arrays.asList();
  static final List<String> EXCLUIR = Arrays.asList();
  // true: so importa, posiciona e grava verificacao.csv (bbox antes/depois), sem malhar nem rodar.
  static final boolean SO_VERIFICAR = false;

  // Posicionamento automatico depois de importar (CADs com origens diferentes), em metros.
  // Valores do trabalho: asa de referencia = asa do .sim base.
  //   x: bordo de ataque (xmin) em REF_XMIN
  //   y: topo da asa (ymax) em REF_YMAX  (o topo e comum as asas; a base do endplate varia)
  //   z: envergadura centrada no plano de simetria Z_SIMETRIA
  static final boolean POSICIONAR = true;
  static final double REF_XMIN = -0.0273643;
  static final double REF_YMAX = 0.357324;
  static final double Z_SIMETRIA = 0.471;

  static final String NOME_ASA = "Asa";
  static final String REPORT_LIFT = "Lift";
  static final String REPORT_DRAG = "Drag";
  // Opcional: se nao existir no .sim, as colunas cp_* ficam vazias.
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

    File[] geoms = new File(GEOM_DIR).listFiles((d, n) -> {
      if (!n.toLowerCase().endsWith(".x_t")) {
        return false;
      }
      String base = n.replaceAll("(?i)\\.x_t$", "");
      return RODAR.isEmpty() ? !EXCLUIR.contains(base) : RODAR.contains(base);
    });
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
        escreverLinha(resumo, nome + "," + csv("ERRO: " + e.getMessage())
            + ",,,,,,,,,,,,,,," + fmt(total) + ",,,,");
      }
    }
    sim.println("Batch concluido. Resumo em " + resumo.getPath());
  }

  String rodarCaso(File geom, String nome, long t0) throws Exception {
    File pasta = new File(OUT_DIR, nome);
    pasta.mkdirs();

    // 1) Trocar a geometria da asa no Subtract, guardando quem usava os contornos da asa
    List<Referencia> refs = coletarReferenciasAsa();
    GeometryPart novaAsa = importarAsa(geom);
    posicionar(novaAsa);
    String bbox = bboxTexto(novaAsa);
    SubtractPartsOperation sub = trocarAsaNoSubtract(novaAsa);

    // 2) Subtract + contorno da asa, depois malha
    sim.getSolution().clearSolution(Solution.Clear.History, Solution.Clear.Fields);
    long tMalha = System.currentTimeMillis();
    sub.execute();
    Boundary contornoAsa = prepararContornoAsa();
    reapontarParaNovaAsa(refs, contornoAsa);
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
    String[] cp = centroDePressao();

    // 5) Imagens e .sim
    for (SceneBase sb : sim.getSceneManager().getObjects()) {
      if (sb instanceof Scene) {
        String arq = nome + "_" + sb.getPresentationName().replaceAll("[^A-Za-z0-9._-]", "_") + ".png";
        ((Scene) sb).printAndWait(new File(pasta, arq).getPath(), 1, IMG_W, IMG_H, true, false);
      }
    }
    sim.saveState(new File(pasta, nome + ".sim").getPath());

    // Ultima iteracao (mesmo criterio da linha Base_manual) e oscilacao na janela final
    double[] ultimo = amostras.getLast();
    double total = (System.currentTimeMillis() - t0) / 1000.0;
    return String.join(",", nome, "ok", parada, String.valueOf(iteracoes), String.valueOf(celulas),
        fmt(dragMeio), fmt(liftMeio), fmt(dragTotal), fmt(downforce), fmt(eficiencia),
        cp[0], cp[1], cp[2], csv(bbox), fmt(tempoMalha), fmt(tempoSolver), fmt(total),
        fmt(ultimo[1] * FATOR_SIMETRIA), fmt(-ultimo[0] * FATOR_SIMETRIA),
        fmt(oscilacao(amostras, 1) * 100), fmt(oscilacao(amostras, 0) * 100));
  }

  // Grava os valores da simulacao manual que ja vem resolvida no .sim base
  // (ultima iteracao, sem media), para comparar com a mesma asa rodada pela macro.
  void registrarBaseManual(File resumo) {
    try {
      int iter = sim.getSimulationIterator().getCurrentIteration();
      if (iter == 0) {
        return;
      }
      for (String l : java.nio.file.Files.readAllLines(resumo.toPath(), java.nio.charset.StandardCharsets.UTF_8)) {
        if (l.startsWith("Base_manual,")) {
          return;
        }
      }
      double liftMeio = sim.getReportManager().getReport(REPORT_LIFT).getReportMonitorValue();
      double dragMeio = sim.getReportManager().getReport(REPORT_DRAG).getReportMonitorValue();
      double dragTotal = dragMeio * FATOR_SIMETRIA;
      double downforce = -liftMeio * FATOR_SIMETRIA;
      String[] cp = centroDePressao();
      long celulas = ((FvRepresentation) sim.getRepresentationManager().getObject("Volume Mesh")).getCellCount();
      escreverLinha(resumo, String.join(",", "Base_manual", "ok", "manual", String.valueOf(iter), String.valueOf(celulas),
          fmt(dragMeio), fmt(liftMeio), fmt(dragTotal), fmt(downforce), fmt(dragTotal != 0 ? downforce / dragTotal : Double.NaN),
          cp[0], cp[1], cp[2], "", "", "", "", fmt(dragTotal), fmt(downforce), "", ""));
    } catch (Exception e) {
      sim.println("Nao foi possivel registrar a simulacao manual: " + e);
    }
  }

  String[] centroDePressao() {
    Report r = sim.getReportManager().has(REPORT_CP) ? sim.getReportManager().getReport(REPORT_CP) : null;
    if (!(r instanceof CenterOfLoadsReport)) {
      return new String[] {"", "", ""};
    }
    Vector3 v = ((CenterOfLoadsReport) r).getNetLocation();
    return new String[] {fmt(v.x), fmt(v.y), fmt(v.z)};
  }

  // ---- Referencias aos contornos da asa (reports e displayers) -------------

  // Um objeto (report ou displayer) que tinha contornos da asa na lista de partes.
  static class Referencia {
    final String rotulo;
    final ClientServerObject dono;
    final List<NamedObject> outros;   // o que nao era da asa (ex.: Block.*), mantido
    Referencia(String rotulo, ClientServerObject dono, List<NamedObject> outros) {
      this.rotulo = rotulo;
      this.dono = dono;
      this.outros = outros;
    }
  }

  boolean ehContornoDaAsa(Object o) {
    return o instanceof Boundary && ((Boundary) o).getPresentationName().startsWith(NOME_ASA + ".");
  }

  Referencia avaliar(String rotulo, ClientServerObject dono, Collection<? extends NamedObject> objs) {
    boolean temAsa = false;
    List<NamedObject> outros = new ArrayList<>();
    for (NamedObject o : objs) {
      if (ehContornoDaAsa(o)) {
        temAsa = true;
      } else {
        outros.add(o);
      }
    }
    return temAsa ? new Referencia(rotulo, dono, outros) : null;
  }

  List<Referencia> coletarReferenciasAsa() {
    List<Referencia> refs = new ArrayList<>();
    for (Report r : sim.getReportManager().getObjects()) {
      Referencia ref = null;
      if (r instanceof ForceReport) {
        ref = avaliar("report " + r.getPresentationName(), r, ((ForceReport) r).getParts().getObjects());
      } else if (r instanceof CenterOfLoadsReport) {
        ref = avaliar("report " + r.getPresentationName(), r, ((CenterOfLoadsReport) r).getParts().getObjects());
      }
      if (ref != null) {
        refs.add(ref);
      }
    }
    for (Scene cena : sim.getSceneManager().getScenes()) {
      for (DisplayerBase d : cena.getDisplayerManager().getObjects()) {
        Referencia ref = null;
        String rot = cena.getPresentationName() + "/" + d.getPresentationName();
        if (d instanceof PartDisplayer) {
          ref = avaliar(rot, d, ((PartDisplayer) d).getInputParts().getObjects());
        } else if (d instanceof ScalarDisplayer) {
          ref = avaliar(rot, d, ((ScalarDisplayer) d).getInputParts().getObjects());
        }
        if (ref != null) {
          refs.add(ref);
        }
      }
    }
    return refs;
  }

  // Depois do Subtract e antes da malha: garante que as superficies da asa no Subtract
  // ("Asa.Asa") estejam num contorno de parede proprio. Reaproveita um contorno antigo
  // da asa (ja do tipo Wall), renomeando para "Asa.Asa", e tira essas superficies de
  // qualquer outro contorno onde o STAR as tenha colocado.
  Boundary prepararContornoAsa() {
    String nomeNovo = NOME_ASA + "." + NOME_ASA;
    MeshPart partSub = null;
    for (GeometryPart gp : sim.get(SimulationPartManager.class).getParts()) {
      if (gp instanceof MeshOperationPart) {
        partSub = (MeshPart) gp;
      }
    }
    if (partSub == null) {
      throw new RuntimeException("peca do Subtract nao encontrada");
    }
    List<PartSurface> supsAsa = new ArrayList<>();
    for (PartSurface s : partSub.getPartSurfaceManager().getPartSurfaces()) {
      if (s.getPresentationName().startsWith(NOME_ASA + ".")) {
        supsAsa.add(s);
      }
    }
    if (supsAsa.isEmpty()) {
      throw new RuntimeException("Subtract nao gerou superficies " + NOME_ASA + ".*");
    }

    Boundary alvo = null;
    Region regiao = null;
    for (Region reg : sim.getRegionManager().getRegions()) {
      for (Boundary b : reg.getBoundaryManager().getBoundaries()) {
        if (b.getPresentationName().equals(nomeNovo) || (alvo == null && ehContornoDaAsa(b))) {
          alvo = b;
          regiao = reg;
        }
      }
    }
    if (alvo == null) {
      throw new RuntimeException("nenhum contorno " + NOME_ASA + ".* na regiao para reaproveitar");
    }
    for (Boundary b : regiao.getBoundaryManager().getBoundaries()) {
      if (b != alvo) {
        List<PartSurface> tirar = new ArrayList<>();
        for (PartSurface s : supsAsa) {
          if (b.getPartSurfaceGroup().has(s)) {
            tirar.add(s);
          }
        }
        if (!tirar.isEmpty()) {
          sim.println("  tirando " + tirar.size() + " superficie(s) da asa de " + b.getPresentationName());
          b.getPartSurfaceGroup().removeObjects(tirar);
        }
      }
    }
    alvo.getPartSurfaceGroup().setObjects(supsAsa);
    alvo.setPresentationName(nomeNovo);
    sim.println("  contorno " + nomeNovo + " <- " + supsAsa.size() + " superficie(s)");
    return alvo;
  }

  // Aponta reports/displayers para o contorno da nova asa e apaga contornos antigos
  // da asa que tenham sobrado vazios na regiao.
  void reapontarParaNovaAsa(List<Referencia> refs, Boundary nova) {
    List<Boundary> sobras = new ArrayList<>();
    for (Region reg : sim.getRegionManager().getRegions()) {
      for (Boundary b : reg.getBoundaryManager().getBoundaries()) {
        if (b != nova && ehContornoDaAsa(b) && b.getPartSurfaceGroup().isEmpty()) {
          sobras.add(b);
        }
      }
    }
    String nomeNovo = nova.getPresentationName();
    for (Referencia ref : refs) {
      List<NamedObject> objs = new ArrayList<>(ref.outros);
      objs.add(nova);
      if (ref.dono instanceof ForceReport) {
        ((ForceReport) ref.dono).getParts().setObjects(objs);
      } else if (ref.dono instanceof CenterOfLoadsReport) {
        ((CenterOfLoadsReport) ref.dono).getParts().setObjects(objs);
      } else if (ref.dono instanceof PartDisplayer) {
        ((PartDisplayer) ref.dono).getInputParts().setObjects(objs);
      } else if (ref.dono instanceof ScalarDisplayer) {
        ((ScalarDisplayer) ref.dono).getInputParts().setObjects(objs);
      }
      sim.println("  " + ref.rotulo + " -> " + nomeNovo);
    }
    if (!sobras.isEmpty()) {
      try {
        sim.deleteObjects(new ArrayList<ClientServerObject>(sobras));
      } catch (Exception e) {
        sim.println("  aviso: nao apagou contornos antigos da asa: " + e);
      }
    }
  }

  // ---- Geometria ------------------------------------------------------------

  // Importa o .x_t, junta todos os corpos numa peca e todas as superficies numa so.
  GeometryPart importarAsa(File geom) {
    SimulationPartManager partes = sim.get(SimulationPartManager.class);
    Set<String> antes = new HashSet<>();
    for (GeometryPart p : partes.getParts()) {
      antes.add(p.getPresentationName());
    }

    // Mesmas opcoes de uma importacao de CAD gravada pela GUI (File > Import > Import Surface Mesh/CAD)
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
  SubtractPartsOperation trocarAsaNoSubtract(GeometryPart novaAsa) {
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

    // Displayers que mostravam a peca/superficies antigas (ex.: Geometry Scene)
    // ficariam vazios ao apagar a peca; guarda quais eram para colocar a nova asa.
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
    // O nome da peca define o nome da superficie no Subtract ("Asa.Asa").
    ((NamedObject) novaAsa).setPresentationName(NOME_ASA);
    return sub;
  }

  // Bounding box na ordem xmin ymin zmin xmax ymax zmax.
  DoubleVector bboxVetor(GeometryPart p) {
    RootDescriptionSource raiz = sim.get(SimulationMeshPartDescriptionSourceManager.class).getRootDescriptionSource();
    return ((MeshPart) p).getDescriptions().getCurrentDescription(raiz).getBoundingBox();
  }

  String bboxTexto(GeometryPart p) {
    try {
      DoubleVector bb = bboxVetor(p);
      StringBuilder sb = new StringBuilder();
      for (int i = 0; i < bb.size(); i++) {
        sb.append(i > 0 ? " " : "").append(fmt(bb.get(i)));
      }
      return sb.toString();
    } catch (Exception e) {
      return "indisponivel";
    }
  }

  // Translada a asa conforme REF_XMIN / REF_YMAX / Z_SIMETRIA (sem rotacao).
  void posicionar(GeometryPart asa) {
    if (!POSICIONAR) {
      return;
    }
    DoubleVector bb = bboxVetor(asa);
    double dx = REF_XMIN - bb.get(0);
    double dy = REF_YMAX - bb.get(4);
    double dz = Z_SIMETRIA - (bb.get(2) + bb.get(5)) / 2.0;
    if (Math.abs(dx) + Math.abs(dy) + Math.abs(dz) < 1e-6) {
      return;
    }
    Units m = (Units) sim.getUnitsManager().getObject("m");
    sim.get(SimulationPartManager.class).translateParts(Arrays.asList(asa), Arrays.asList(dx, dy, dz),
        Arrays.asList(m, m, m), sim.getCoordinateSystemManager().getLabCoordinateSystem());
    sim.println(String.format(Locale.US, "  posicionada: dx=%.4f dy=%.4f dz=%.4f m", dx, dy, dz));
  }

  void verificar(File[] geoms) {
    File saida = new File(OUT_DIR, "verificacao.csv");
    escreverLinha(saida, "geometria,bbox_original,bbox_posicionada,dx_dy_dz");
    for (File geom : geoms) {
      String nome = geom.getName().replaceAll("(?i)\\.x_t$", "");
      try {
        GeometryPart asa = importarAsa(geom);
        DoubleVector a = bboxVetor(asa);
        String antes = bboxTexto(asa);
        posicionar(asa);
        DoubleVector d = bboxVetor(asa);
        String desloc = fmt(d.get(0) - a.get(0)) + " " + fmt(d.get(1) - a.get(1)) + " " + fmt(d.get(2) - a.get(2));
        escreverLinha(saida, nome + "," + csv(antes) + "," + csv(bboxTexto(asa)) + "," + csv(desloc));
        sim.deleteObjects(new ArrayList<ClientServerObject>(Arrays.asList(asa)));
      } catch (Exception e) {
        escreverLinha(saida, nome + "," + csv("ERRO: " + e.getMessage()) + ",,");
      }
    }
    sim.println("Verificacao concluida: " + saida.getPath());
  }

  // ---- Utilitarios ----------------------------------------------------------

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

  // Campo CSV entre aspas, numa linha so (mensagens de erro do STAR tem varias linhas).
  static String csv(String s) {
    String limpo = String.valueOf(s).replace("\"", "'").replaceAll("[\\r\\n]+", " ")
        .replaceAll("-{3,}", " ").replaceAll("\\s+", " ").trim();
    return "\"" + limpo + "\"";
  }

  static void escreverLinha(File f, String linha) {
    try (PrintWriter w = new PrintWriter(new OutputStreamWriter(new FileOutputStream(f, true), "UTF-8"))) {
      w.println(linha);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }
}
