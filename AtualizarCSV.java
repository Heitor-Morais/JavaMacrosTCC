// Simcenter STAR-CCM+ macro: AtualizarCSV.java
// Abra o .sim JA RESOLVIDO de uma asa (ex.: SimulationBatches\Asa_Dianteira_V.10\Asa_Dianteira_V.10.sim)
// e rode esta macro. Ela le o historico dos monitores de Lift e Drag e, com o mesmo criterio
// do BatchAsas (amostras a cada 10 iteracoes, ultimas 100), calcula a media e a oscilacao
// das forcas na janela final. A media das ultimas 100 iteracoes, que e o valor comparado
// com a macro, e mostrada na mensagem de execucao. Na linha dessa asa em resumo.csv, a
// macro preenche as colunas drag_ultimo_N, downforce_ultimo_N, osc_drag_pct e osc_lift_pct.
// Nao roda o solver e nao altera o .sim.
package macro;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.*;

import star.common.*;
import star.base.report.*;

public class AtualizarCSV extends StarMacro {

  static final String RESUMO = "C:\\Users\\heito\\ccmp\\starccm-mcp\\SimulationBatches\\resumo.csv";
  static final String REPORT_LIFT = "Lift";
  static final String REPORT_DRAG = "Drag";
  static final int JANELA = 100;
  static final int PASSO = 10;
  static final double FATOR_SIMETRIA = 2.0;

  Simulation sim;

  @Override
  public void execute() {
    sim = getActiveSimulation();
    String nome = sim.getPresentationName();

    double[] lift = historico(REPORT_LIFT);
    double[] drag = historico(REPORT_DRAG);
    double[] amLift = amostras(lift);
    double[] amDrag = amostras(drag);

    double dragUlt = drag[drag.length - 1] * FATOR_SIMETRIA;
    double dfUlt = -lift[lift.length - 1] * FATOR_SIMETRIA;
    double oscDrag = oscilacao(amDrag) * 100;
    double oscLift = oscilacao(amLift) * 100;
    sim.println(String.format(Locale.US,
        "%s: %d iteracoes no historico | drag media %.6g / ultimo %.6g N | downforce media %.6g / ultimo %.6g N | osc drag %.3g%% lift %.3g%%",
        nome, lift.length, media(amDrag) * FATOR_SIMETRIA, dragUlt, -media(amLift) * FATOR_SIMETRIA, dfUlt, oscDrag, oscLift));

    try {
      File f = new File(RESUMO);
      List<String> linhas = new ArrayList<>(Files.readAllLines(f.toPath(), StandardCharsets.UTF_8));
      String novos = String.join(",", fmt(dragUlt), fmt(dfUlt), fmt(oscDrag), fmt(oscLift));
      boolean achou = false;
      for (int i = 0; i < linhas.size(); i++) {
        String l = linhas.get(i);
        if (l.startsWith(nome + ",")) {
          // As 4 ultimas colunas sao as que esta macro preenche: troca o que houver nelas.
          int corte = l.length();
          for (int k = 0; k < 4; k++) {
            corte = l.lastIndexOf(',', corte - 1);
          }
          linhas.set(i, l.substring(0, corte + 1) + novos);
          achou = true;
        }
      }
      if (!achou) {
        sim.println("Linha '" + nome + "' nao encontrada em " + RESUMO + " (nada alterado).");
        return;
      }
      Files.write(f.toPath(), linhas, StandardCharsets.UTF_8);
      sim.println("resumo.csv atualizado para " + nome);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  double[] historico(String report) {
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

  // Valores a cada PASSO iteracoes nas ultimas JANELA, terminando na ultima (igual ao BatchAsas)
  static double[] amostras(double[] y) {
    int n = Math.min(JANELA / PASSO, (y.length + PASSO - 1) / PASSO);
    double[] a = new double[n];
    for (int k = 0; k < n; k++) {
      a[k] = y[y.length - 1 - k * PASSO];
    }
    return a;
  }

  static double media(double[] a) {
    double s = 0;
    for (double v : a) {
      s += v;
    }
    return s / a.length;
  }

  static double oscilacao(double[] a) {
    double min = Double.MAX_VALUE, max = -Double.MAX_VALUE;
    for (double v : a) {
      min = Math.min(min, v);
      max = Math.max(max, v);
    }
    return (max - min) / Math.abs(media(a));
  }

  static String fmt(double v) {
    return String.format(Locale.US, "%.6g", v);
  }
}
