function o = coklu_sim(p, h, tauc, lider_ivme, T_stop, dt)
% Çok araç dinleyen CACC konvoyunun zaman alanı modeli (RK4, sabit adım).
% Parametreler coklu_frekans ile aynı; ek olarak d0, L, u_min, u_max, v0.
% lider_ivme: [t u] matrisi. p.lider_lag = 0 (varsayılan): liderde a = u, CACC_konvoy.slx gibi.
% p.lider_lag = 1: lider de tau*a' + a = u ile sürülür.
% tauc = 0 veya tauc >= dt olmalı. Gecikmeli sinyaller geçmişten doğrusal enterpole edilir.

if tauc > 0 && tauc < dt, error('tauc 0 veya en az dt olmalı.'); end
lider_lag = isfield(p, 'lider_lag') && p.lider_lag;
N = p.N;  M = numel(p.kv);
t = (0:dt:T_stop)';  n = numel(t);
d_ss = p.d0 + h*p.v0;

x = zeros(3*(N + 1), 1);                  % [p; v; a], 1 = lider
x(1:N + 1) = -(0:N)' * (d_ss + p.L);
x(N + 2:2*N + 2) = p.v0;

hV = zeros(n, N + 1);  hU = zeros(n, N + 1);    % gecikme için geçmiş
o.tout = t;
o.a_log = zeros(n, N + 1);  o.v_log = zeros(n, N + 1);
o.d_log = zeros(n, N);      o.e_log = zeros(n, N);
ta = lider_ivme(:, 1);  aa = lider_ivme(:, 2);

for k = 1:n
    [~, u] = turev(t(k), x, k);
    v = x(N + 2:2*N + 2);  a = x(2*N + 3:end);
    if ~lider_lag, a(1) = u(1); end
    hV(k, :) = v';  hU(k, :) = u';
    if p.ff_u == 0, hU(k, :) = a'; end
    d = x(1:N) - x(2:N + 1) - p.L;
    o.a_log(k, :) = a';  o.v_log(k, :) = v';
    o.d_log(k, :) = d';  o.e_log(k, :) = (d - p.d0 - h*v(2:end))';
    if k == n, break; end
    k1 = turev(t(k), x, k);
    k2 = turev(t(k) + dt/2, x + dt/2*k1, k);
    k3 = turev(t(k) + dt/2, x + dt/2*k2, k);
    k4 = turev(t(k) + dt, x + dt*k3, k);
    x = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end

    function [dx, u] = turev(ts, xs, ks)
        pos = xs(1:N + 1);  vs = xs(N + 2:2*N + 2);  as = xs(2*N + 3:end);
        u = zeros(N + 1, 1);
        u(1) = lider(ts);
        for i = 1:N
            arac = i + 1;
            d_i = pos(arac - 1) - pos(arac) - p.L;
            ui = p.Kp*(d_i - p.d0 - h*vs(arac)) + p.kv(1)*(vs(arac - 1) - vs(arac));
            for j = 1:min(i, M)
                [vg, ug] = gecikmeli(arac - j, ts, ks, vs, u, as);
                if j >= 2, ui = ui + p.kv(j)*(vg - vs(arac)); end
                ui = ui + p.ka(j)*ug;
            end
            u(arac) = min(max(ui, p.u_min), p.u_max);
        end
        if lider_lag
            dx = [vs; as; (u - as)/p.tau];
        else
            dx = [vs; u(1); as(2:end); 0; (u(2:end) - as(2:end))/p.tau];   % lider: a = u
        end
    end

    function [vg, ug] = gecikmeli(no, ts, ks, vs, u, as)
        if tauc == 0
            vg = vs(no);
            if p.ff_u == 1 || no == 1, ug = u(no); else, ug = as(no); end
            return
        end
        tg = ts - tauc;
        if tg <= 0
            vg = p.v0;  ug = 0;  return       % t < 0: seyir hâli
        end
        i0 = min(floor(tg/dt + 1e-9) + 1, ks - 1);
        i0 = max(i0, 1);
        if ks == 1, vg = hV(1, no); ug = hU(1, no); return; end
        oran = (tg - t(i0))/dt;
        vg = hV(i0, no) + oran*(hV(i0 + 1, no) - hV(i0, no));
        ug = hU(i0, no) + oran*(hU(i0 + 1, no) - hU(i0, no));
    end

    function a = lider(ts)
        i = find(ta <= ts, 1, 'last');
        if isempty(i), a = 0; else, a = aa(i); end
    end
end
