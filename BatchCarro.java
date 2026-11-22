// Simcenter STAR-CCM+ macro: BatchCarro.java  (testada no STAR-CCM+ 21.04)
//
// O QUE FAZ
//   Batch do carro inteiro. Para cada montagem Parasolid (.x_t) de GEOM_DIR:
//   1) importa a montagem (copia antes para um nome sem acento: o leitor Parasolid falha com "ção");
//   2) compara peca a peca com o que esta no .sim base (nome + bounding box):
//        - igual       -> mantem a peca do .sim (rodas divididas, radiador poroso etc. ficam como estao)
//        - diferente   -> troca no Subtract mantendo o nome da peca antiga (ex.: asas)
//        - sumiu       -> tira do Subtract (ex.: duto que nao existe no CAD novo)
//        - nova        -> entra no Subtract com contorno de parede proprio
//   3) refaz o Subtract, religa contornos/reports/cenas/controles de malha, gera a malha,
//      roda com o mesmo criterio de parada das asas e grava resumo.csv + PNGs + .sim por caso.
//
// REQUISITOS DO .sim BASE (o que estiver aberto quando a macro roda)
//   1. Simulacao do carro ja montada e funcionando com uma das geometrias (malha, fisica,
//      contornos, reports e cenas configurados). A macro so troca geometria; nao cria setup.
//   2. Uma unica operacao Subtract (dominio menos o carro) e a regiao do ar criada a partir dela.
//      Os contornos dessa regiao seguem o padrao do STAR "<Peca>.<Superficie>"; o dominio e "Block.*".
//   3. Reports de forca REPORT_LIFT (direcao z, z para cima) e REPORT_DRAG (direcao x) aplicados
//      as paredes do carro. REPORT_MASSA_RAD e opcional. REPORT_MOMENTO e criado pela macro.
//   4. Meio carro com simetria em y (forcas x FATOR_SIMETRIA; carro inteiro: FATOR_SIMETRIA = 1).
//   5. Pecas da base com superficies divididas a mao ou regioes proprias (rodas com rotacao,
//      radiador poroso...) vao em NAO_TROCAVEIS: elas precisam ter a MESMA geometria em todos os CADs
//      (sao mantidas da base). Se uma delas mudar, a macro para com erro em vez de trocar errado.
//
// REQUISITOS DOS CADs (.x_t)
//   - Mesma montagem, unidades e sistema de coordenadas do CAD usado no .sim base (nada e
//     transladado nem rotacionado). Pecas que mudaram precisam se sobrepor a versao antiga
//     (bounding box com IoU > IOU_MIN) para serem reconhecidas como "a mesma peca modificada".
//   - Pecas que o CAD exporta separadas mas a base tem juntas: listar em COMBINAR.
//
// ESPECIFICO DO CARRO DO TRABALHO (ajustar para outro carro)
//   Nomes dos reports, COMBINAR, NAO_TROCAVEIS, GRUPOS (prefixos dos contornos por componente),
//   posicao dos eixos e do chao (balanco aerodinamico), RADIADOR e TAMANHO_BASE.
//   Opcoes que nao se aplicam sao ignoradas (ex.: sem peca RADIADOR, nada e feito no radiador;
//   grupo sem contorno correspondente sai com 0).
//
// USO
//   Ajuste a secao "Configuracao", rode antes com SO_VERIFICAR = true (so troca a geometria e
//   grava verificacao.txt com o que foi mantido/trocado/removido/adicionado) e depois rode de verdade,
//   pela GUI (File > Macro > Play Macro, com o .sim base aberto) ou em batch:
//     starccm+ -batch BatchCarro.java -np <nucleos> <CarroBase.sim>
//   (com licenca Power-on-Demand: acrescente -power -podkey <CHAVE>)
//   Cada caso parte do estado deixado pelo anterior; para comparar casos de forma limpa,
//   rode um caso por vez a partir do .sim base (lista RODAR).
package macro;

import java.io.*;
import java.nio.file.*;
import java.text.Normalizer;
import java.util.*;

import star.common.*;
import star.base.neo.*;
import star.base.report.*;
import star.flow.*;
import star.meshing.*;
import star.vis.*;

public class BatchCarro extends StarMacro {

  // ---- Configuracao -------------------------------------------------------
  // Pasta com os .x_t e pasta de saida (podem ser a mesma).
  static final String GEOM_DIR = "C:\\caminho\\para\\geometrias";
  static final String OUT_DIR = "C:\\caminho\\para\\resultados";
  // Ordem de execucao (nomes sem .x_t). Vazia = todas em ordem alfabetica, menos EXCLUIR.
  // Escreva sem acento: a comparacao com o nome do arquivo ignora acentos.
  static final List<String> RODAR = Arrays.asList();
  static final List<String> EXCLUIR = Arrays.asList();
  // true: importa, compara, troca e refaz o Subtract/contornos, grava verificacao.txt; sem malha, sem solver, sem salvar.
  static final boolean SO_VERIFICAR = false;

  static final String REPORT_LIFT = "Litf";   // nome como esta no .sim (direcao z)
  static final String REPORT_DRAG = "Drag";
  static final String REPORT_MASSA_RAD = "Mass Flow 1";
  static final String REPORT_MOMENTO = "Momento eixo dianteiro";

  // Pecas que o CAD exporta separadas e o .sim base tem juntas (combineMeshParts na macro gravada).
  static final String[][] COMBINAR = {{"ES16_E1_PAE_FiltroAr", "ES16_E1_PAE_FlangeAdmiss"}};
  // Pecas com superficies divididas a mao no .sim (patches/porosa): nao da para trocar automaticamente.
  static final List<String> NAO_TROCAVEIS = Arrays.asList("Pneu Simplificado", "Radiador 330x300 sim");
  static final double TOL_BBOX = 2e-4;        // m
  static final double IOU_MIN = 0.3;          // sobreposicao minima de bbox para considerar "a mesma peca, modificada"

  // Balanco aerodinamico: momento em y em torno do contato do pneu dianteiro com o chao.
  // Centros das rodas = sistemas "Cartesian RD" e "Cartesian 1" do .sim; chao = zmin do bloco.
  static final double X_EIXO_DIANT = -0.6128056;
  static final double X_EIXO_TRAS = 0.9274127;
  static final double Z_CHAO = 1.058786;

  // Grupos para a divisao das forcas por componente (prefixo do nome do contorno).
  static final String[][] GRUPOS = {
      {"asa_diant", "Asa Dianteira."},
      {"asa_tras", "Asa Traseira."},
      {"rodas", "Pneu Simplificado."},
      {"undertray", "ES16_E1_PAE_Undertray"}};

  // Parada (igual as asas): min MIN_ITER, max MAX_ITER; para antes se Lift e Drag
  // variarem menos que TOL nas ultimas JANELA iteracoes.
  static final int MIN_ITER = 300;
  static final int MAX_ITER = 1000;
  static final int JANELA = 100;
  static final int PASSO = 10;
  static final double TOL = 0.005;
  static final double FATOR_SIMETRIA = 2.0;

  // Estabilidade (com os padroes do STAR, 0.7/0.3/0.8, a solucao do carro divergiu ~iteracao 150).
  static final boolean ESTABILIZAR = true;
  static final double URF_VELOCIDADE = 0.5;
  static final double URF_PRESSAO = 0.2;
  static final double URF_TURBULENCIA = 0.6;
  static final boolean REMEDIAR_CELULAS = true;   // modelo Cell Quality Remediation
  // Radiador poroso sem camadas de prisma (nos dois lados das interfaces): a divergencia
  // comecava em celulas ruins na lateral do radiador, colada ao sidepod.
  static final boolean RADIADOR_SEM_PRISMAS = true;
  static final String RADIADOR = "Radiador 330x300 sim";
  static final String CONTROLE_RADIADOR = "Radiador sem prismas";
  // Base size da malha do ar (malhador que usa o Subtract). <= 0 mantem o valor do .sim.
  // Os tamanhos de superficie, prismas e o controle volumetrico sao relativos ao base size, entao
  // tudo refina junto. No trabalho: 0.3 m -> ~4.2 M celulas; 0.235 m -> ~5.45 M celulas.
  static final double TAMANHO_BASE = 0.235;
  // Aborta o caso se Drag ou Lift passarem disso (meio carro, N): solucao divergiu.
  static final double FORCA_ABSURDA = 1.0e5;

  static final int IMG_W = 1920;
  static final int IMG_H = 1080;
  // -------------------------------------------------------------------------

  Simulation sim;
  PrintWriter log;

  @Override
  public void execute() {
    sim = getActiveSimulation();
    new File(OUT_DIR).mkdirs();
    List<File> geoms = listarGeometrias();
    if (geoms.isEmpty()) {
      sim.println("Nenhum .x_t encontrado em " + GEOM_DIR);
      return;
    }

    File resumo = new File(OUT_DIR, "resumo.csv");
    if (!SO_VERIFICAR && !resumo.exists()) {
      StringBuilder h = new StringBuilder("geometria,status,parada,iteracoes,celulas,"
          + "drag_N,downforce_N,eficiencia,pct_dianteira,pct_traseira,");
      for (String[] g : GRUPOS) {
        h.append(g[0]).append("_downforce_N,").append(g[0]).append("_drag_N,");
      }
      h.append("resto_downforce_N,resto_drag_N,vazao_radiador_kg_s,trocas,"
          + "tempo_malha_s,tempo_solver_s,tempo_total_s,drag_ultimo_N,downforce_ultimo_N,osc_drag_pct,osc_lift_pct");
      escreverLinha(resumo, h.toString());
    }
    try {
      log = new PrintWriter(new OutputStreamWriter(new FileOutputStream(
          new File(OUT_DIR, SO_VERIFICAR ? "verificacao.txt" : "log_batch.txt"), true), "UTF-8"), true);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }

    for (File geom : geoms) {
      String nome = geom.getName().replaceAll("(?i)\\.x_t$", "");
      String nomeArq = semAcento(nome);
      p("=== " + nome + " ===");
      long t0 = System.currentTimeMillis();
      try {
        String linha = rodarCaso(geom, nomeArq, t0);
        if (!SO_VERIFICAR) {
          escreverLinha(resumo, linha);
        }
      } catch (Exception e) {
        p("ERRO em " + nome + ": " + e);
        for (StackTraceElement el : e.getStackTrace()) {
          if (el.getClassName().contains("BatchCarro")) {
            p("    em " + el);
          }
        }
        if (!SO_VERIFICAR) {
          double total = (System.currentTimeMillis() - t0) / 1000.0;
          StringBuilder vaz = new StringBuilder();
          for (int i = 0; i < 2 * GRUPOS.length + 15; i++) {
            vaz.append(',');
          }
          escreverLinha(resumo, nomeArq + "," + csv("ERRO: " + e.getMessage()) + vaz + fmt(total) + ",,,,");
        }
      }
    }
    p(SO_VERIFICAR ? "Verificacao concluida (nada foi salvo)." : "Batch concluido. Resumo em " + resumo.getPath());
    log.close();
  }

  List<File> listarGeometrias() {
    List<File> out = new ArrayList<>();
    File[] todos = new File(GEOM_DIR).listFiles((d, n) -> n.toLowerCase().endsWith(".x_t"));
    if (todos == null) {
      return out;
    }
    Arrays.sort(todos);
    if (RODAR.isEmpty()) {
      for (File f : todos) {
        if (!EXCLUIR.contains(f.getName().replaceAll("(?i)\\.x_t$", ""))) {
          out.add(f);
        }
      }
      return out;
    }
    for (String r : RODAR) {
      for (File f : todos) {
        String b = f.getName().replaceAll("(?i)\\.x_t$", "");
        // compara sem acento e em NFC, para nao depender de como o nome foi gravado no disco
        if (semAcento(b).equalsIgnoreCase(semAcento(r))) {
          out.add(f);
        }
      }
    }
    return out;
  }

  String rodarCaso(File geom, String nome, long t0) throws Exception {
    File pasta = new File(OUT_DIR, nome);
    pasta.mkdirs();

    // 1) Geometria
    String trocas = atualizarGeometria(geom);
    prepararMomento();
    if (SO_VERIFICAR) {
      return "";
    }

    // 2) Malha
    sim.getSolution().clearSolution(Solution.Clear.History, Solution.Clear.Fields);
    radiadorSemPrismas();
    ajustarTamanhoBase();
    long tMalha = System.currentTimeMillis();
    sim.getMeshPipelineController().generateVolumeMesh();
    double tempoMalha = (System.currentTimeMillis() - tMalha) / 1000.0;
    long celulas = ((FvRepresentation) sim.getRepresentationManager().getObject("Volume Mesh")).getCellCount();
    p("  malha: " + celulas + " celulas em " + fmt(tempoMalha) + " s");

    // 3) Solver
    aplicarEstabilidade();
    sim.getSolution().initializeSolution();
    ForceReport lift = (ForceReport) sim.getReportManager().getReport(REPORT_LIFT);
    ForceReport drag = (ForceReport) sim.getReportManager().getReport(REPORT_DRAG);
    LinkedList<double[]> amostras = new LinkedList<>();
    int amostrasJanela = Math.max(1, JANELA / PASSO);
    SimulationIterator it = sim.getSimulationIterator();
    String parada = "limite";
    long tSolver = System.currentTimeMillis();
    while (it.getCurrentIteration() < MAX_ITER) {
      it.step(Math.min(PASSO, MAX_ITER - it.getCurrentIteration()));
      double l = lift.getReportMonitorValue(), d = drag.getReportMonitorValue();
      amostras.add(new double[] {l, d});
      if (amostras.size() > amostrasJanela) {
        amostras.removeFirst();
      }
      if (Double.isNaN(l) || Double.isNaN(d) || Math.abs(l) > FORCA_ABSURDA || Math.abs(d) > FORCA_ABSURDA) {
        parada = "divergiu";
        break;
      }
      if (it.getCurrentIteration() >= MIN_ITER && amostras.size() == amostrasJanela
          && estavel(amostras, 0) && estavel(amostras, 1)) {
        parada = "convergiu";
        break;
      }
    }
    double tempoSolver = (System.currentTimeMillis() - tSolver) / 1000.0;
    int iteracoes = it.getCurrentIteration();
    p("  solver: " + parada + " em " + iteracoes + " iteracoes, " + fmt(tempoSolver) + " s");
    if (parada.equals("divergiu")) {
      throw new RuntimeException("solucao divergiu na iteracao " + iteracoes + " (forcas > " + FORCA_ABSURDA + " N)");
    }

    // 4) Resultados (totais = media da janela final; divisao por componente = ultima iteracao)
    double liftMeio = media(amostras, 0);
    double dragMeio = media(amostras, 1);
    double dragTotal = dragMeio * FATOR_SIMETRIA;
    double downforce = -liftMeio * FATOR_SIMETRIA;
    double eficiencia = dragTotal != 0 ? downforce / dragTotal : Double.NaN;

    String pctD = "", pctT = "";
    try {
      double my = sim.getReportManager().getReport(REPORT_MOMENTO).getReportMonitorValue();
      double liftUlt = lift.getReportMonitorValue();
      double cargaTras = my / (X_EIXO_TRAS - X_EIXO_DIANT);   // meio carro, para baixo
      double frac = cargaTras / (-liftUlt);
      pctT = fmt(100 * frac);
      pctD = fmt(100 * (1 - frac));
    } catch (Exception e) {
      p("  aviso: balanco nao calculado: " + e);
    }

    Map<String, double[]> grupos = new LinkedHashMap<>();
    for (String[] g : GRUPOS) {
      grupos.put(g[0], new double[2]);
    }
    grupos.put("resto", new double[2]);
    for (Object o : drag.getParts().getObjects()) {
      String n = ((NamedObject) o).getPresentationName();
      String chave = "resto";
      for (String[] g : GRUPOS) {
        if (n.startsWith(g[1])) {
          chave = g[0];
        }
      }
      double[] acc = grupos.get(chave);
      // getValue(contorno) devolve o vetor forca naquele contorno; projeta nas direcoes dos reports
      double fz = lift.getValue((ClientServerObject) o).z;   // Litf: direcao (0,0,1)
      double fx = drag.getValue((ClientServerObject) o).x;   // Drag: direcao (1,0,0)
      if (Double.isNaN(fz) || Double.isNaN(fx)) {
        // interfaces do radiador devolvem NaN aqui; a forca delas ja esta nas faces do lado do ar
        p("  (sem valor por contorno em " + n + ")");
        continue;
      }
      acc[0] += -fz * FATOR_SIMETRIA;
      acc[1] += fx * FATOR_SIMETRIA;
    }
    StringBuilder colGrupos = new StringBuilder();
    for (double[] acc : grupos.values()) {
      colGrupos.append(fmt(acc[0])).append(',').append(fmt(acc[1])).append(',');
    }
    String vazao = "";
    if (sim.getReportManager().has(REPORT_MASSA_RAD)) {
      vazao = fmt(sim.getReportManager().getReport(REPORT_MASSA_RAD).getReportMonitorValue());
    }

    // 5) Imagens e .sim
    for (SceneBase sb : sim.getSceneManager().getObjects()) {
      if (sb instanceof Scene) {
        String arq = nome + "_" + sb.getPresentationName().replaceAll("[^A-Za-z0-9._-]", "_") + ".png";
        try {
          ((Scene) sb).printAndWait(new File(pasta, arq).getPath(), 1, IMG_W, IMG_H, true, false);
        } catch (Exception e) {
          p("  aviso: imagem " + arq + " falhou: " + e);
        }
      }
    }
    sim.saveState(new File(pasta, nome + ".sim").getPath());

    double[] ultimo = amostras.getLast();
    double total = (System.currentTimeMillis() - t0) / 1000.0;
    return nome + ",ok," + parada + "," + iteracoes + "," + celulas + ","
        + fmt(dragTotal) + "," + fmt(downforce) + "," + fmt(eficiencia) + "," + pctD + "," + pctT + ","
        + colGrupos + vazao + "," + csv(trocas) + ","
        + fmt(tempoMalha) + "," + fmt(tempoSolver) + "," + fmt(total) + ","
        + fmt(ultimo[1] * FATOR_SIMETRIA) + "," + fmt(-ultimo[0] * FATOR_SIMETRIA) + ","
        + fmt(oscilacao(amostras, 1) * 100) + "," + fmt(oscilacao(amostras, 0) * 100);
  }

  // ===========================================================================
  // Troca de geometria
  // ===========================================================================

  String atualizarGeometria(File geom) throws IOException {
    SimulationPartManager pm = sim.get(SimulationPartManager.class);
    SubtractPartsOperation sub = subtract();
    MeshOperationPart partSub = pecaSubtract();
    Region regiao = regiaoDe(partSub);

    List<GeometryPart> base = pecasCarro(pm.getParts());
    List<GeometryPart> novas = importar(geom);
    combinarComoNaBase(novas);

    // --- casamento -------------------------------------------------------
    Map<GeometryPart, GeometryPart> iguais = new LinkedHashMap<>();
    List<GeometryPart> restBase = new ArrayList<>(base);
    List<GeometryPart> restNovas = new ArrayList<>(novas);
    for (GeometryPart b : base) {
      for (GeometryPart n : restNovas) {
        if (normalizar(b.getPresentationName()).equals(normalizar(n.getPresentationName())) && mesmaForma(b, n)) {
          iguais.put(b, n);
          restNovas.remove(n);
          restBase.remove(b);
          break;
        }
      }
    }
    Map<GeometryPart, GeometryPart> trocas = new LinkedHashMap<>();
    while (true) {
      double melhor = IOU_MIN;
      GeometryPart mb = null, mn = null;
      for (GeometryPart b : restBase) {
        for (GeometryPart n : restNovas) {
          double iou = iou(bbox(b), bbox(n));
          if (iou > melhor) {
            melhor = iou;
            mb = b;
            mn = n;
          }
        }
      }
      if (mb == null) {
        break;
      }
      trocas.put(mb, mn);
      restBase.remove(mb);
      restNovas.remove(mn);
    }
    List<GeometryPart> removidas = restBase;
    List<GeometryPart> adicionadas = restNovas;

    StringBuilder resumo = new StringBuilder();
    p("  iguais: " + iguais.size() + " peca(s)");
    for (Map.Entry<GeometryPart, GeometryPart> e : trocas.entrySet()) {
      String mesmaGeo = mesmaForma(e.getKey(), e.getValue()) ? " (mesma bbox)" : "";
      p("  troca: " + e.getKey().getPresentationName() + " <- " + e.getValue().getPresentationName() + mesmaGeo);
      resumo.append("troca ").append(e.getKey().getPresentationName()).append(" <- ").append(e.getValue().getPresentationName()).append("; ");
    }
    for (GeometryPart b : removidas) {
      p("  removida: " + b.getPresentationName() + " (nao existe no CAD novo)");
      resumo.append("removida ").append(b.getPresentationName()).append("; ");
    }
    for (GeometryPart n : adicionadas) {
      p("  nova: " + n.getPresentationName() + " bbox=" + fmtBbox(bbox(n)));
      resumo.append("nova ").append(n.getPresentationName()).append("; ");
    }
    for (GeometryPart b : concat(trocas.keySet(), removidas)) {
      if (b instanceof CompositePart || NAO_TROCAVEIS.contains(b.getPresentationName())) {
        throw new RuntimeException("a peca " + b.getPresentationName()
            + " mudou no CAD, mas tem superficies divididas/montagem no .sim; troca automatica nao suportada");
      }
    }
    for (GeometryPart n : adicionadas) {
      if (n instanceof CompositePart) {
        throw new RuntimeException("montagem nova " + n.getPresentationName() + " nao suportada");
      }
    }

    // --- quem usa contornos (reports, cenas) e controles de malha ----------
    Set<String> prefixosMexidos = new HashSet<>();
    for (GeometryPart b : concat(trocas.keySet(), removidas)) {
      prefixosMexidos.add(b.getPresentationName() + ".");
    }
    List<Dono> donos = coletarDonos(regiao);
    Set<String> contornosRemovidos = new HashSet<>();
    for (Boundary bd : regiao.getBoundaryManager().getBoundaries()) {
      for (GeometryPart b : removidas) {
        if (bd.getPresentationName().startsWith(b.getPresentationName() + ".")) {
          contornosRemovidos.add(chave(regiao, bd));
        }
      }
    }
    aplicarDonos(donos, contornosRemovidos, Collections.<Boundary>emptyList());
    List<CustomMeshControl> controlesMexidos = tirarDosControles(prefixosMexidos);
    Map<PartDisplayer, List<GeometryPart>> dispGeo = displayersDeGeometria(trocas.keySet(), removidas);

    // --- Subtract ------------------------------------------------------------
    for (Map.Entry<GeometryPart, GeometryPart> e : trocas.entrySet()) {
      sub.getInputGeometryObjects().addPart(e.getValue());
    }
    for (GeometryPart n : adicionadas) {
      sub.getInputGeometryObjects().addPart(n);
    }
    List<GeometryPart> sair = concat(trocas.keySet(), removidas);
    if (!sair.isEmpty()) {
      sub.getInputGeometryObjects().removeParts(sair);
    }
    List<ClientServerObject> apagar = new ArrayList<>(sair);
    apagar.addAll(iguais.values());
    // nomes antigos guardados antes de apagar
    Map<GeometryPart, String> nomeAntigo = new LinkedHashMap<>();
    for (Map.Entry<GeometryPart, GeometryPart> e : trocas.entrySet()) {
      nomeAntigo.put(e.getValue(), e.getKey().getPresentationName());
    }
    if (!apagar.isEmpty()) {
      sim.deleteObjects(apagar);
    }
    for (Map.Entry<GeometryPart, String> e : nomeAntigo.entrySet()) {
      e.getKey().setPresentationName(e.getValue());
    }
    for (GeometryPart n : adicionadas) {
      String limpo = normalizar(n.getPresentationName());
      if (!pm.has(limpo)) {
        n.setPresentationName(limpo);
      }
    }
    // cena de geometria: coloca as pecas novas onde estavam as antigas
    for (Map.Entry<PartDisplayer, List<GeometryPart>> e : dispGeo.entrySet()) {
      List<NamedObject> objs = new ArrayList<>(e.getKey().getInputParts().getObjects());
      for (GeometryPart antiga : e.getValue()) {
        for (Map.Entry<GeometryPart, GeometryPart> t : trocas.entrySet()) {
          if (t.getKey() == antiga) {
            objs.add(t.getValue());
          }
        }
      }
      e.getKey().getInputParts().setObjects(objs);
    }

    if (trocas.isEmpty() && removidas.isEmpty() && adicionadas.isEmpty()) {
      p("  geometria igual a do .sim: nada a trocar");
      return "nenhuma (igual a base)";
    }

    sub.execute();

    // --- contornos ------------------------------------------------------------
    List<String> prefixos = new ArrayList<>();
    for (GeometryPart n : trocas.values()) {
      prefixos.add(n.getPresentationName() + ".");
    }
    List<String> prefixosNovas = new ArrayList<>();
    for (GeometryPart n : adicionadas) {
      prefixos.add(n.getPresentationName() + ".");
      prefixosNovas.add(n.getPresentationName() + ".");
    }
    List<Boundary> criados = ligarSuperficies(partSub, regiao, prefixos);
    criados.addAll(ligarOrfas(partSub, regiao));
    apagarContornosVazios(regiao, prefixosMexidos, prefixos);
    aplicarDonos(donos, contornosRemovidos, criados);
    religarControles(controlesMexidos, partSub, trocas.values(), adicionadas);

    // conferencia
    ForceReport drag = (ForceReport) sim.getReportManager().getReport(REPORT_DRAG);
    p("  report " + REPORT_DRAG + ": " + drag.getParts().getObjects().size() + " contornos");
    for (String pref : prefixos) {
      int n = 0;
      for (Object o : drag.getParts().getObjects()) {
        if (((NamedObject) o).getPresentationName().startsWith(pref)) {
          n++;
        }
      }
      p("    " + pref + "* no " + REPORT_DRAG + ": " + n);
    }
    return resumo.toString().trim();
  }

  List<GeometryPart> importar(File geom) throws IOException {
    SimulationPartManager pm = sim.get(SimulationPartManager.class);
    Set<GeometryPart> antes = new HashSet<>(pm.getParts());
    File tmp = new File(OUT_DIR, "_importando.x_t");
    Files.copy(geom.toPath(), tmp.toPath(), StandardCopyOption.REPLACE_EXISTING);
    try {
      // Mesmas opcoes de uma importacao de CAD gravada pela GUI
      sim.get(PartImportManager.class).importCadPart2(resolvePath(tmp.getPath()), "SharpEdges", 30.0, 2, true, 1.0E-5, true, false, false, false, true, true, false, false, false, CadImportUtilities.SimplifyAssemblyNameRetentionOption.IMMEDIATE_ASSEMBLY_NAME);
    } finally {
      tmp.delete();
    }
    List<GeometryPart> novas = new ArrayList<>();
    for (GeometryPart gp : pm.getParts()) {
      if (!antes.contains(gp)) {
        novas.add(gp);
      }
    }
    if (novas.isEmpty()) {
      throw new RuntimeException("importacao nao criou nenhuma peca (arquivo Parasolid ilegivel?)");
    }
    p("  importadas " + novas.size() + " peca(s)");
    return novas;
  }

  void combinarComoNaBase(List<GeometryPart> novas) {
    for (String[] regra : COMBINAR) {
      MeshPart alvo = null;
      List<MeshPart> outras = new ArrayList<>();
      for (GeometryPart gp : novas) {
        String n = gp.getPresentationName();
        if (gp instanceof MeshPart && n.startsWith(regra[0]) && alvo == null) {
          alvo = (MeshPart) gp;
        }
      }
      for (int i = 1; i < regra.length; i++) {
        for (GeometryPart gp : novas) {
          if (gp instanceof MeshPart && gp.getPresentationName().startsWith(regra[i])) {
            outras.add((MeshPart) gp);
          }
        }
      }
      if (alvo != null && !outras.isEmpty()) {
        sim.get(MeshPartFactory.class).combineMeshParts(alvo, new ArrayList<>(outras));
        novas.removeAll(outras);
        p("  combinada: " + alvo.getPresentationName() + " (+" + outras.size() + ")");
      }
    }
  }

  // Pecas de nivel superior que nao sao blocos nem o Subtract.
  static List<GeometryPart> pecasCarro(Collection<GeometryPart> todas) {
    List<GeometryPart> out = new ArrayList<>();
    for (GeometryPart gp : todas) {
      if (!(gp instanceof SimpleBlockPart) && !(gp instanceof MeshOperationPart)) {
        out.add(gp);
      }
    }
    return out;
  }

  // Liga cada superficie nova do Subtract ("Peca.Sup") ao contorno de mesmo nome; se nao
  // existir, ao primeiro contorno da peca; se a peca nao tiver contorno, cria um (parede).
  List<Boundary> ligarSuperficies(MeshOperationPart partSub, Region regiao, List<String> prefixos) {
    List<Boundary> criados = new ArrayList<>();
    for (String pref : prefixos) {
      List<PartSurface> sups = new ArrayList<>();
      for (PartSurface s : partSub.getPartSurfaceManager().getPartSurfaces()) {
        if (s.getPresentationName().startsWith(pref)) {
          sups.add(s);
        }
      }
      if (sups.isEmpty()) {
        p("  aviso: o Subtract nao gerou superficies " + pref + "* (peca fora do dominio?)");
        continue;
      }
      Boundary principal = null;
      for (Boundary b : regiao.getBoundaryManager().getBoundaries()) {
        if (principal == null && b.getPresentationName().startsWith(pref) && !(b instanceof InterfaceBoundary)) {
          principal = b;
        }
      }
      Map<Boundary, List<PartSurface>> destino = new LinkedHashMap<>();
      for (PartSurface s : sups) {
        Boundary alvo = regiao.getBoundaryManager().has(s.getPresentationName())
            ? regiao.getBoundaryManager().getBoundary(s.getPresentationName()) : principal;
        if (alvo == null) {
          alvo = regiao.getBoundaryManager().createEmptyBoundary(s.getPresentationName());
          criados.add(alvo);
          principal = alvo;
          p("  contorno criado: " + alvo.getPresentationName());
        }
        destino.computeIfAbsent(alvo, k -> new ArrayList<>()).add(s);
      }
      // tira essas superficies de qualquer outro contorno onde o STAR as tenha posto
      for (Boundary b : regiao.getBoundaryManager().getBoundaries()) {
        if (b instanceof InterfaceBoundary) {
          continue;
        }
        List<PartSurface> tirar = new ArrayList<>();
        for (PartSurface s : sups) {
          if (b.getPartSurfaceGroup().has(s) && !(destino.containsKey(b) && destino.get(b).contains(s))) {
            tirar.add(s);
          }
        }
        if (!tirar.isEmpty()) {
          b.getPartSurfaceGroup().removeObjects(tirar);
        }
      }
      for (Map.Entry<Boundary, List<PartSurface>> e : destino.entrySet()) {
        e.getKey().getPartSurfaceGroup().setObjects(e.getValue());
        p("  contorno " + e.getKey().getPresentationName() + " <- " + e.getValue().size() + " superficie(s)");
      }
    }
    return criados;
  }

  // Superficies do Subtract que nao estao em nenhum contorno (ex.: "Faces", criada onde uma peca
  // removida encostava em outra). Sem contorno o malhador para; vira parede e entra nos reports.
  List<Boundary> ligarOrfas(MeshOperationPart partSub, Region regiao) {
    List<Boundary> criados = new ArrayList<>();
    for (PartSurface s : partSub.getPartSurfaceManager().getPartSurfaces()) {
      boolean temDono = false;
      for (Boundary b : regiao.getBoundaryManager().getBoundaries()) {
        if (b.getPartSurfaceGroup().has(s)) {
          temDono = true;
          break;
        }
      }
      if (temDono) {
        continue;
      }
      String nome = "Sem dono." + s.getPresentationName();
      Boundary b = regiao.getBoundaryManager().has(nome)
          ? regiao.getBoundaryManager().getBoundary(nome) : regiao.getBoundaryManager().createEmptyBoundary(nome);
      List<PartSurface> sups = new ArrayList<>(b.getPartSurfaceGroup().getObjects());
      sups.add(s);
      b.getPartSurfaceGroup().setObjects(sups);
      if (!criados.contains(b)) {
        criados.add(b);
      }
      p("  aviso: superficie " + s.getPresentationName() + " sem contorno -> parede " + nome + " (entra nos reports)");
    }
    return criados;
  }

  void apagarContornosVazios(Region regiao, Set<String> prefixosAntigos, List<String> prefixosNovos) {
    Set<String> todos = new HashSet<>(prefixosAntigos);
    todos.addAll(prefixosNovos);
    List<ClientServerObject> vazios = new ArrayList<>();
    for (Boundary b : regiao.getBoundaryManager().getBoundaries()) {
      if (b instanceof InterfaceBoundary || !b.getPartSurfaceGroup().isEmpty()) {
        continue;
      }
      for (String pref : todos) {
        if (b.getPresentationName().startsWith(pref)) {
          vazios.add(b);
          break;
        }
      }
    }
    if (!vazios.isEmpty()) {
      try {
        sim.deleteObjects(vazios);
        p("  contornos vazios apagados: " + vazios.size());
      } catch (Exception e) {
        p("  aviso: nao apagou contornos vazios: " + e);
      }
    }
  }

  // ---- Reports e displayers: guardados por nome de contorno -------------------

  static class Dono {
    final ClientServerObject obj;
    final String rotulo;
    final List<NamedObject> outros = new ArrayList<>();   // o que nao e contorno: mantido como esta
    final List<String> contornos = new ArrayList<>();     // "regiao|contorno"
    boolean geral;                                       // tinha quase todas as paredes do carro
    Dono(ClientServerObject obj, String rotulo) {
      this.obj = obj;
      this.rotulo = rotulo;
    }
  }

  static String chave(Region r, Boundary b) {
    return r.getPresentationName() + "|" + b.getPresentationName();
  }

  List<Dono> coletarDonos(Region regiaoCarro) {
    Set<Boundary> paredesCarro = new HashSet<>();
    for (Boundary b : regiaoCarro.getBoundaryManager().getBoundaries()) {
      if (!b.getPresentationName().startsWith("Block.") && !(b instanceof InterfaceBoundary)) {
        paredesCarro.add(b);
      }
    }
    List<Dono> donos = new ArrayList<>();
    for (Report r : sim.getReportManager().getObjects()) {
      Collection<? extends NamedObject> objs = partesDe(r);
      if (objs != null) {
        donos.add(montarDono(r, "report " + r.getPresentationName(), objs, paredesCarro));
      }
    }
    for (Scene cena : sim.getSceneManager().getScenes()) {
      for (DisplayerBase d : cena.getDisplayerManager().getObjects()) {
        Collection<? extends NamedObject> objs = partesDe(d);
        if (objs != null) {
          donos.add(montarDono(d, cena.getPresentationName() + "/" + d.getPresentationName(), objs, paredesCarro));
        }
      }
    }
    return donos;
  }

  Dono montarDono(ClientServerObject obj, String rotulo, Collection<? extends NamedObject> objs, Set<Boundary> paredesCarro) {
    Dono d = new Dono(obj, rotulo);
    int nParedes = 0;
    for (NamedObject o : objs) {
      if (o instanceof Boundary) {
        Boundary b = (Boundary) o;
        d.contornos.add(chave(b.getRegion(), b));
        if (paredesCarro.contains(b)) {
          nParedes++;
        }
      } else {
        d.outros.add(o);
      }
    }
    d.geral = !paredesCarro.isEmpty() && nParedes >= 0.5 * paredesCarro.size();
    return d;
  }

  static Collection<? extends NamedObject> partesDe(Object o) {
    if (o instanceof ForceReport) {
      return ((ForceReport) o).getParts().getObjects();
    } else if (o instanceof MomentReport) {
      return ((MomentReport) o).getParts().getObjects();
    } else if (o instanceof MassFlowReport) {
      return ((MassFlowReport) o).getParts().getObjects();
    } else if (o instanceof AreaAverageReport) {
      return ((AreaAverageReport) o).getParts().getObjects();
    } else if (o instanceof PartDisplayer) {
      return ((PartDisplayer) o).getInputParts().getObjects();
    } else if (o instanceof ScalarDisplayer) {
      return ((ScalarDisplayer) o).getInputParts().getObjects();
    } else if (o instanceof VectorDisplayer) {
      return ((VectorDisplayer) o).getInputParts().getObjects();
    }
    return null;
  }

  static void definirPartes(Object o, List<NamedObject> objs) {
    if (o instanceof ForceReport) {
      ((ForceReport) o).getParts().setObjects(objs);
    } else if (o instanceof MomentReport) {
      ((MomentReport) o).getParts().setObjects(objs);
    } else if (o instanceof MassFlowReport) {
      ((MassFlowReport) o).getParts().setObjects(objs);
    } else if (o instanceof AreaAverageReport) {
      ((AreaAverageReport) o).getParts().setObjects(objs);
    } else if (o instanceof PartDisplayer) {
      ((PartDisplayer) o).getInputParts().setObjects(objs);
    } else if (o instanceof ScalarDisplayer) {
      ((ScalarDisplayer) o).getInputParts().setObjects(objs);
    } else if (o instanceof VectorDisplayer) {
      ((VectorDisplayer) o).getInputParts().setObjects(objs);
    }
  }

  void aplicarDonos(List<Dono> donos, Set<String> excluir, List<Boundary> extras) {
    Map<String, Boundary> porChave = new HashMap<>();
    for (Region r : sim.getRegionManager().getRegions()) {
      for (Boundary b : r.getBoundaryManager().getBoundaries()) {
        porChave.put(chave(r, b), b);
      }
    }
    for (Dono d : donos) {
      List<NamedObject> objs = new ArrayList<>(d.outros);
      int faltando = 0;
      for (String k : d.contornos) {
        if (excluir.contains(k)) {
          continue;
        }
        Boundary b = porChave.get(k);
        if (b != null) {
          objs.add(b);
        } else {
          faltando++;
        }
      }
      if (d.geral) {
        objs.addAll(extras);
      }
      definirPartes(d.obj, objs);
      if (faltando > 0) {
        p("  aviso: " + d.rotulo + ": " + faltando + " contorno(s) nao encontrado(s)");
      }
    }
  }

  // ---- Controles de malha ------------------------------------------------------

  // Tira dos controles de superficie/volume os objetos das pecas que vao sair (proxy da peca e
  // superficies "Peca.*" do Subtract). Devolve os controles mexidos, para religar depois.
  List<CustomMeshControl> tirarDosControles(Set<String> prefixos) {
    List<CustomMeshControl> mexidos = new ArrayList<>();
    for (MeshOperation op : sim.get(MeshOperationManager.class).getObjects()) {
      if (!(op instanceof AutoMeshOperation)) {
        continue;
      }
      for (CustomMeshControl c : ((AutoMeshOperation) op).getCustomMeshControls().getObjects()) {
        if (!(c instanceof SurfaceCustomMeshControl)) {
          continue;
        }
        SurfaceCustomMeshControl sc = (SurfaceCustomMeshControl) c;
        List<GeometryObject> manter = new ArrayList<>();
        boolean mexeu = false;
        for (GeometryObject o : sc.getGeometryObjects().getObjects()) {
          String n = o.getPresentationName();
          boolean daPeca = false;
          for (String pref : prefixos) {
            String nomePeca = pref.substring(0, pref.length() - 1);
            if (n.equals(nomePeca) || n.startsWith(pref)) {
              daPeca = true;
            }
          }
          if (daPeca) {
            mexeu = true;
          } else {
            manter.add(o);
          }
        }
        if (mexeu) {
          sc.getGeometryObjects().setObjects(manter);
          mexidos.add(sc);
        }
      }
    }
    return mexidos;
  }

  void religarControles(List<CustomMeshControl> mexidos, MeshOperationPart partSub,
      Collection<GeometryPart> trocadas, List<GeometryPart> adicionadas) {
    // Pecas trocadas voltam para os controles de onde sairam.
    for (CustomMeshControl c : mexidos) {
      SurfaceCustomMeshControl sc = (SurfaceCustomMeshControl) c;
      List<GeometryObject> objs = new ArrayList<>(sc.getGeometryObjects().getObjects());
      for (GeometryPart n : trocadas) {
        objs.add(partSub.getOrCreateProxyForObject(n));
        objs.addAll(superficiesDe(partSub, n.getPresentationName() + "."));
      }
      sc.getGeometryObjects().setObjects(objs);
      p("  controle " + sc.getPresentationName() + " religado");
    }
    // Pecas novas entram no controle de superficie com mais objetos (o da carroceria).
    if (!adicionadas.isEmpty()) {
      SurfaceCustomMeshControl maior = null;
      for (MeshOperation op : sim.get(MeshOperationManager.class).getObjects()) {
        if (op instanceof AutoMeshOperation) {
          for (CustomMeshControl c : ((AutoMeshOperation) op).getCustomMeshControls().getObjects()) {
            if (c instanceof SurfaceCustomMeshControl && (maior == null
                || ((SurfaceCustomMeshControl) c).getGeometryObjects().getObjects().size()
                > maior.getGeometryObjects().getObjects().size())) {
              maior = (SurfaceCustomMeshControl) c;
            }
          }
        }
      }
      if (maior != null) {
        List<GeometryObject> objs = new ArrayList<>(maior.getGeometryObjects().getObjects());
        for (GeometryPart n : adicionadas) {
          objs.add(partSub.getOrCreateProxyForObject(n));
          objs.addAll(superficiesDe(partSub, n.getPresentationName() + "."));
        }
        maior.getGeometryObjects().setObjects(objs);
        p("  pecas novas no controle " + maior.getPresentationName());
      }
    }
  }

  static List<PartSurface> superficiesDe(MeshOperationPart partSub, String pref) {
    List<PartSurface> out = new ArrayList<>();
    for (PartSurface s : partSub.getPartSurfaceManager().getPartSurfaces()) {
      if (s.getPresentationName().startsWith(pref)) {
        out.add(s);
      }
    }
    return out;
  }

  // Displayers da cena de geometria que mostram as pecas CAD que vao sair.
  Map<PartDisplayer, List<GeometryPart>> displayersDeGeometria(Collection<GeometryPart> trocadas, List<GeometryPart> removidas) {
    Map<PartDisplayer, List<GeometryPart>> out = new LinkedHashMap<>();
    List<GeometryPart> saem = concat(trocadas, removidas);
    for (Scene cena : sim.getSceneManager().getScenes()) {
      for (DisplayerBase d : cena.getDisplayerManager().getObjects()) {
        if (!(d instanceof PartDisplayer)) {
          continue;
        }
        PartDisplayer pd = (PartDisplayer) d;
        List<NamedObject> manter = new ArrayList<>();
        List<GeometryPart> tinha = new ArrayList<>();
        for (Object o : pd.getInputParts().getObjects()) {
          GeometryPart dono = null;
          if (o instanceof GeometryPart && saem.contains(o)) {
            dono = (GeometryPart) o;
          } else if (o instanceof PartSurface && saem.contains(((PartSurface) o).getPart())) {
            dono = (GeometryPart) ((PartSurface) o).getPart();
          }
          if (dono != null) {
            if (!tinha.contains(dono)) {
              tinha.add(dono);
            }
          } else {
            manter.add((NamedObject) o);
          }
        }
        if (!tinha.isEmpty()) {
          pd.getInputParts().setObjects(manter);
          out.put(pd, tinha);
        }
      }
    }
    return out;
  }

  // ---- Acesso ao .sim -------------------------------------------------------------

  SubtractPartsOperation subtract() {
    for (MeshOperation op : sim.get(MeshOperationManager.class).getObjects()) {
      if (op instanceof SubtractPartsOperation) {
        return (SubtractPartsOperation) op;
      }
    }
    throw new RuntimeException("operacao Subtract nao encontrada no .sim");
  }

  MeshOperationPart pecaSubtract() {
    for (GeometryPart gp : sim.get(SimulationPartManager.class).getParts()) {
      if (gp instanceof MeshOperationPart) {
        return (MeshOperationPart) gp;
      }
    }
    throw new RuntimeException("peca do Subtract nao encontrada");
  }

  Region regiaoDe(GeometryPart gp) {
    for (Region r : sim.getRegionManager().getRegions()) {
      if (r.getPartGroup().getObjects().contains(gp)) {
        return r;
      }
    }
    throw new RuntimeException("regiao do Subtract nao encontrada");
  }

  // Em cada malhador que envolve o radiador (o do Subtract e o da regiao porosa), poe as
  // superficies do radiador num controle proprio com prismas desligados e tira-as dos outros controles.
  void radiadorSemPrismas() {
    if (!RADIADOR_SEM_PRISMAS) {
      return;
    }
    MeshOperationPart partSub = pecaSubtract();
    for (MeshOperation op : sim.get(MeshOperationManager.class).getObjects()) {
      if (!(op instanceof AutoMeshOperation)) {
        continue;
      }
      AutoMeshOperation am = (AutoMeshOperation) op;
      List<GeometryObject> alvo = new ArrayList<>();
      Collection<GeometryObject> entradas = am.getInputGeometryObjects().getObjects();
      if (entradas.contains(partSub)) {
        alvo.addAll(superficiesDe(partSub, RADIADOR + "."));
      } else {
        for (GeometryObject go : entradas) {
          if (go.getPresentationName().equals(RADIADOR)) {
            alvo.add(go);
          }
        }
      }
      if (alvo.isEmpty()) {
        continue;
      }
      SurfaceCustomMeshControl ctrl = null;
      for (CustomMeshControl c : am.getCustomMeshControls().getObjects()) {
        if (c instanceof SurfaceCustomMeshControl && c.getPresentationName().equals(CONTROLE_RADIADOR)) {
          ctrl = (SurfaceCustomMeshControl) c;
        }
      }
      for (CustomMeshControl c : am.getCustomMeshControls().getObjects()) {
        if (c instanceof SurfaceCustomMeshControl && c != ctrl) {
          SurfaceCustomMeshControl sc = (SurfaceCustomMeshControl) c;
          List<GeometryObject> manter = new ArrayList<>();
          boolean mexeu = false;
          for (GeometryObject o : sc.getGeometryObjects().getObjects()) {
            String n = o.getPresentationName();
            if (alvo.contains(o) || n.equals(RADIADOR) || n.startsWith(RADIADOR + ".")) {
              mexeu = true;
            } else {
              manter.add(o);
            }
          }
          if (mexeu) {
            sc.getGeometryObjects().setObjects(manter);
          }
        }
      }
      if (ctrl == null) {
        ctrl = am.getCustomMeshControls().createSurfaceControl();
        ctrl.setPresentationName(CONTROLE_RADIADOR);
      }
      ctrl.getGeometryObjects().setObjects(alvo);
      ctrl.getCustomConditions().get(star.prismmesher.PartsCustomizePrismMesh.class)
          .getCustomPrismOptions().setSelected(star.prismmesher.PartsCustomPrismsOption.Type.DISABLE);
      p("  " + am.getPresentationName() + ": radiador sem prismas (" + alvo.size() + " objeto(s))");
    }
  }

  void ajustarTamanhoBase() {
    if (TAMANHO_BASE <= 0) {
      return;
    }
    MeshOperationPart partSub = pecaSubtract();
    Units metro = (Units) sim.getUnitsManager().getObject("m");
    for (MeshOperation op : sim.get(MeshOperationManager.class).getObjects()) {
      if (op instanceof AutoMeshOperation
          && ((AutoMeshOperation) op).getInputGeometryObjects().getObjects().contains(partSub)) {
        ((AutoMeshOperation) op).getDefaultValues().get(BaseSize.class).setValueAndUnits(TAMANHO_BASE, metro);
        p("  " + op.getPresentationName() + ": base size = " + TAMANHO_BASE + " m");
      }
    }
  }

  void aplicarEstabilidade() {
    if (!ESTABILIZAR) {
      return;
    }
    try {
      star.segregatedflow.SegregatedFlowSolver sf = sim.getSolverManager().getSolver(star.segregatedflow.SegregatedFlowSolver.class);
      sf.getVelocitySolver().setUrf(URF_VELOCIDADE);
      sf.getPressureSolver().setUrf(URF_PRESSAO);
      p("  URF velocidade=" + URF_VELOCIDADE + " pressao=" + URF_PRESSAO);
    } catch (Exception e) {
      p("  aviso: URF do escoamento nao alterado: " + e);
    }
    try {
      sim.getSolverManager().getSolver(star.kwturb.KwTurbSolver.class).setUrf(URF_TURBULENCIA);
      p("  URF turbulencia=" + URF_TURBULENCIA);
    } catch (Exception e) {
      p("  aviso: URF da turbulencia nao alterado: " + e);
    }
    if (REMEDIAR_CELULAS) {
      for (Object o : sim.getContinuumManager().getObjects()) {
        if (o instanceof PhysicsContinuum) {
          PhysicsContinuum pc = (PhysicsContinuum) o;
          try {
            if (!pc.getModelManager().has("Cell Quality Remediation")) {
              pc.enable(star.metrics.CellQualityRemediationModel.class);
            }
            p("  Cell Quality Remediation ligado em " + pc.getPresentationName());
          } catch (Exception e) {
            p("  aviso: Cell Quality Remediation nao ligado: " + e);
          }
        }
      }
    }
  }

  // Momento em y em torno do contato do pneu dianteiro (cria uma vez; as partes acompanham o Drag).
  void prepararMomento() {
    ForceReport drag = (ForceReport) sim.getReportManager().getReport(REPORT_DRAG);
    MomentReport m;
    if (sim.getReportManager().has(REPORT_MOMENTO)) {
      m = (MomentReport) sim.getReportManager().getReport(REPORT_MOMENTO);
    } else {
      m = sim.getReportManager().createReport(MomentReport.class);
      m.setPresentationName(REPORT_MOMENTO);
    }
    Units metro = (Units) sim.getUnitsManager().getObject("m");
    Units adim = (Units) sim.getUnitsManager().getObject("");
    m.getOrigin().setComponentsAndUnits(X_EIXO_DIANT, 0.0, Z_CHAO, metro);
    m.getDirection().setComponentsAndUnits(0.0, 1.0, 0.0, adim);
    m.getParts().setObjects(new ArrayList<NamedObject>(drag.getParts().getObjects()));
  }

  // ---- Geometria: comparacao -----------------------------------------------------

  // Remove o sufixo de duplicata do STAR (" 2") e o nome da montagem ("^ES16_..."), que muda entre setups.
  static String normalizar(String n) {
    return n.replaceAll("\\^.*$", "").replaceAll("( \\d+)+$", "").trim();
  }

  boolean mesmaForma(GeometryPart a, GeometryPart b) {
    if (a instanceof CompositePart || b instanceof CompositePart) {
      if (!(a instanceof CompositePart && b instanceof CompositePart)) {
        return false;
      }
      List<GeometryPart> fa = new ArrayList<>(((CompositePart) a).getChildParts().getParts());
      List<GeometryPart> fb = new ArrayList<>(((CompositePart) b).getChildParts().getParts());
      if (fa.size() != fb.size()) {
        return false;
      }
      for (GeometryPart x : fa) {
        boolean achou = false;
        for (GeometryPart y : fb) {
          if (normalizar(x.getPresentationName()).equals(normalizar(y.getPresentationName())) && mesmaForma(x, y)) {
            achou = true;
            break;
          }
        }
        if (!achou) {
          return false;
        }
      }
      return true;
    }
    double[] ba = bbox(a), bb = bbox(b);
    if (ba == null || bb == null) {
      return false;
    }
    for (int i = 0; i < 6; i++) {
      if (Math.abs(ba[i] - bb[i]) > TOL_BBOX) {
        return false;
      }
    }
    return true;
  }

  double[] bbox(GeometryPart gp) {
    RootDescriptionSource raiz = sim.get(SimulationMeshPartDescriptionSourceManager.class).getRootDescriptionSource();
    double[] r = null;
    List<GeometryPart> folhas = new ArrayList<>();
    folhas(gp, folhas);
    for (GeometryPart f : folhas) {
      try {
        DoubleVector v = ((MeshPart) f).getDescriptions().getCurrentDescription(raiz).getBoundingBox();
        if (r == null) {
          r = new double[] {v.get(0), v.get(1), v.get(2), v.get(3), v.get(4), v.get(5)};
        } else {
          for (int i = 0; i < 3; i++) {
            r[i] = Math.min(r[i], v.get(i));
            r[i + 3] = Math.max(r[i + 3], v.get(i + 3));
          }
        }
      } catch (Exception e) {
        // peca sem descricao: ignora
      }
    }
    return r;
  }

  static void folhas(GeometryPart gp, List<GeometryPart> out) {
    if (gp instanceof CompositePart) {
      for (GeometryPart c : ((CompositePart) gp).getChildParts().getParts()) {
        folhas(c, out);
      }
    } else if (gp instanceof MeshPart) {
      out.add(gp);
    }
  }

  static double iou(double[] a, double[] b) {
    if (a == null || b == null) {
      return 0;
    }
    double inter = 1, va = 1, vb = 1;
    for (int i = 0; i < 3; i++) {
      double lo = Math.max(a[i], b[i]), hi = Math.min(a[i + 3], b[i + 3]);
      if (hi <= lo) {
        return 0;
      }
      inter *= hi - lo;
      va *= Math.max(a[i + 3] - a[i], 1e-9);
      vb *= Math.max(b[i + 3] - b[i], 1e-9);
    }
    return inter / (va + vb - inter);
  }

  // ---- Utilitarios ------------------------------------------------------------------

  @SafeVarargs
  static List<GeometryPart> concat(Collection<GeometryPart>... cs) {
    List<GeometryPart> out = new ArrayList<>();
    for (Collection<GeometryPart> c : cs) {
      out.addAll(c);
    }
    return out;
  }

  void p(String s) {
    sim.println(s);
    if (log != null) {
      log.println(s);
    }
  }

  static String semAcento(String s) {
    return Normalizer.normalize(s, Normalizer.Form.NFD).replaceAll("\\p{M}", "");
  }

  static String fmtBbox(double[] b) {
    if (b == null) {
      return "-";
    }
    StringBuilder sb = new StringBuilder();
    for (int i = 0; i < b.length; i++) {
      sb.append(i > 0 ? " " : "").append(String.format(Locale.US, "%.4f", b[i]));
    }
    return sb.toString();
  }

  static boolean estavel(List<double[]> amostras, int k) {
    return media(amostras, k) != 0 && oscilacao(amostras, k) < TOL;
  }

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
